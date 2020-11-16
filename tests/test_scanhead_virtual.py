import unittest
import math

import numpy as np
from migen import *
from litex.soc.cores.spi import SPIMaster

from hexastorm.controller import Machine
from hexastorm.core import Scanhead
import hexastorm.board as board

class FakeSpi():
    def __init__(self, spimaster):
        self.spimaster = spimaster
    
    def xfer(self, byte_sent):
        byte_received = []
        for byte in byte_sent:
            yield self.spimaster.mosi.eq(byte)
            yield self.spimaster.length.eq(8)
            yield self.spimaster.start.eq(1)
            yield
            yield self.spimaster.start.eq(0)
            yield
            while (yield self.spimaster.done) == 0: yield
            byte_received.append((yield self.spimaster.miso))
        return byte_received


def checkpin(pin, value=0):
    '''check value of pin'''
    timeout = 0
    while (yield pin) != value:
        timeout += 1
        if timeout>100:
            raise Exception(f"Pin doesnt change state")
        yield

def getstate(fsm):
    '''retrieves encoding of state'''
    return fsm.decoding[(yield fsm.state)]

def checkenterstate(fsm, state):
    '''checks if state is entered'''
    timeout  = 0
    # NOTE: state state is dangerous notation not the same
    while (fsm.decoding[(yield fsm.state)] != state):
        timeout += 1
        if timeout>100:
            raise Exception(f"Reached {fsm.decoding[(yield fsm.state)]} not {state}")
        yield

class TestMachine():
    ''' helper class for the test machine '''
    def __init__(self):
        sh = Scanhead
        self.clocks = {"sys": 20, 'clk100':10}
        self.ticksinfacet = 18
        self.laserticks = 4
        sh.MEMDEPTH = 8
        sh.CHUNKSIZE = 1
        sh.VARIABLES['CRYSTAL_HZ']= round(self.ticksinfacet*sh.VARIABLES['FACETS']
                                          *sh.VARIABLES['RPM']/60)
        sh.VARIABLES['LASER_HZ'] = sh.VARIABLES['CRYSTAL_HZ']/self.laserticks
        sh.VARIABLES['SPINUP_TIME'] = 10/sh.VARIABLES['CRYSTAL_HZ']
        sh.VARIABLES['STABLE_TIME'] = 30/sh.VARIABLES['CRYSTAL_HZ']
        sh.VARIABLES['START%'] = 2/self.ticksinfacet
        scanbits = 2
        Scanhead.VARIABLES['END%'] = (self.laserticks*scanbits)/self.ticksinfacet + sh.VARIABLES['START%']
        self.flash()
        
    def flash(self):
        self.sh = Scanhead(board.Platform(), test=True)
        self.sh.submodules.master = SPIMaster(self.sh.spi, data_width=8, sys_clk_freq=100e6,
                                              spi_clk_freq=5e6, with_csr=False)
        self.spi = FakeSpi(self.sh.master)
    
    @property
    def single_line(self):
        '''
        return if system is in single line mode
        '''
        return self.sh.VARIABLES['SINGLE_LINE']

    @single_line.setter
    def single_line(self, val):
        '''
        set system in single line mode
        '''
        assert isinstance(val, bool)
        Scanhead.VARIABLES['SINGLE_LINE'] = val
        self.flash()

    @property
    def single_facet(self):
        '''
        return if system is in single line mode
        '''
        return self.sh.VARIABLES['SINGLE_FACET']

    @single_facet.setter
    def single_facet(self, val):
        '''
        set system in single line mode
        '''
        assert isinstance(val, bool)
        Scanhead.VARIABLES['SINGLE_FACET'] = val
        self.flash()

    def checkreply(self, command, data_expected):
        '''command is sent and checked against result
        '''
        data_received = (yield from self.spi.xfer([command]))[0]
        try:
            assert data_received == data_expected
        except AssertionError:
            state = data_received
            errors = [int(i) for i in list('{0:0b}'.format(state&0b11111))]
            errors.reverse()
            if max(errors)>0:
                error_string = ''
                for idx, val in enumerate(errors):
                    if val>0:
                        error = list(self.sh.ERRORS._asdict())[idx]
                        error_string += error + ' '
            else:
                error_string = 'None'
            machinestate = list(self.sh.STATES._asdict())[state>>5]
            raise Exception(f"State {machinestate} and errors {error_string} not expected")

    def state(self, errors=[], state=None):
        ''' 
        gets a state encoding
        error: list as there can be multiple
        '''
        if state is None: self.sh.STATES.STOP
        errorstate = 0
        for error in errors: errorstate += pow(2, error)
        val = errorstate+(state<<5)
        return val

    def get_state(self, byte=None):
        '''
        places state and error bits in dict
        '''
        if byte is None: raise Exception("should not occur")
        errors = [int(i) for i in list('{0:0b}'.format(byte&0b111111))]
        errors.reverse()
        return {'statebits': byte>>5, 'errorbits': errors}

    def checkline(self, bitlst):
        yield from checkenterstate(self.sh.laserfsm, 'READ_INSTRUCTION')
        yield from checkenterstate(self.sh.laserfsm, 'DATA_RUN')
        yield
        for idx, bit in enumerate(bitlst):
            assert (yield self.sh.lasercnt) == self.laserticks-1
            assert (yield self.sh.scanbit) == idx+1
            for _ in range(self.laserticks):
                assert (yield self.sh.laser0) == bit
                yield

    def forcewrite(self, data, maxtrials=10):
        '''' the question is --> how many ticks are within a transaction'''
        state = self.get_state((yield from self.spi.xfer([data]))[0])
        assert state['statebits'] in [self.sh.STATES.STOP, self.sh.STATES.START]
        trials = 0
        while state['errorbits'][self.sh.ERRORS.MEMFULL]:
           state = self.get_state((yield from self.spi.xfer([data]))[0])
           trials += 1
           if trials>maxtrials:
                raise Exception(f"More than {maxtrials} required to write to memory")

    def writeline(self, bitlst, bitorder = 'little'):
        '''
        writes bitlist to memory
        if bitlst is empty --> stop command is sent
        '''
        if len(bitlst) == 0:
            bytelst = [self.sh.INSTRUCTIONS.STOP]
        else:
            assert len(bitlst) == self.sh.BITSINSCANLINE
            assert max(bitlst) <= 1
            assert min(bitlst) >= 0
            bytelst = np.packbits(bitlst, bitorder=bitorder).tolist()
            bytelst = [self.sh.INSTRUCTIONS.SCAN] + bytelst
        bytelst.reverse()
        #NOTE: chunksize of 1 only supported but this code does support it!
        for _ in range(math.ceil(len(bytelst)/self.sh.CHUNKSIZE)):
            yield from self.forcewrite(self.sh.COMMANDS.WRITE_L)
            for _ in range(self.sh.CHUNKSIZE):
                try:
                    byte = bytelst.pop()
                except IndexError:
                    byte = 0
                yield from self.forcewrite(byte)
        yield from checkenterstate(self.sh.receiver, 'WRITE')
        yield from checkenterstate(self.sh.receiver, 'IDLE')

class TestScanhead(unittest.TestCase):
    ''' Virtual test for scanhead'''
    def setUp(self):
        self.tm = TestMachine()

    def _test_decorator(singleline=False, singlefacet=False, simulatediode=False):
        def nested_dec(func):
            def functie(self):
                if singlefacet: self.tm.single_facet = True
                if singleline: self.tm.single_line = True
                if simulatediode:
                    class DiodeSimulator(Module):
                        def __init__(self, sh):
                            self.submodules.sh = sh
                            diodecounter = Signal(max=sh.ticksinfacet)
                            self.sync += (If(diodecounter == sh.ticksinfacet,
                                            diodecounter.eq(0)
                                           ).
                                         Elif(diodecounter>sh.ticksinfacet-3,
                                            sh.photodiode.eq(~(1&(sh.poly_en==0)&(sh.laser0==1))),
                                            diodecounter.eq(diodecounter + 1)
                                         ).
                                         Else(diodecounter.eq(diodecounter + 1),
                                            sh.photodiode.eq(1)
                                         ))
                    run_simulation(DiodeSimulator(self.tm.sh), [func(self)], clocks=self.tm.clocks)
                else:
                    run_simulation(self.tm.sh, [func(self)], clocks=self.tm.clocks)
                if singleline: self.tm.single_line = False
                if singlefacet: self.tm.single_facet = False
            return functie
        return nested_dec
    
    @_test_decorator()
    def test_pwmgeneration(self):
        ''' verify generation of polygon pulse'''
        yield from checkpin(self.tm.sh.poly_pwm, value=0)
        yield from checkpin(self.tm.sh.poly_pwm, value=1)

    @_test_decorator(simulatediode=True)
    def test_testmodes(self):
        ''' verify four test modes; laser, motor, line and photodiode'''
        test_commands = [self.tm.sh.COMMANDS.LASERTEST,
                            self.tm.sh.COMMANDS.MOTORTEST,
                            self.tm.sh.COMMANDS.LINETEST,
                            self.tm.sh.COMMANDS.PHOTODIODETEST]
        states = [self.tm.sh.STATES.LASERTEST,
                    self.tm.sh.STATES.MOTORTEST,
                    self.tm.sh.STATES.LINETEST,
                    self.tm.sh.STATES.PHOTODIODETEST]
        for idx, test_command in enumerate(test_commands):
            yield from self.tm.checkreply(self.tm.sh.COMMANDS.STATUS, self.tm.state(state=self.tm.sh.STATES.STOP))
            yield from self.tm.checkreply(test_command, self.tm.state(state=self.tm.sh.STATES.STOP))
            if test_command == self.tm.sh.COMMANDS.MOTORTEST:
                yield from checkpin(self.tm.sh.laser0, value=0)
                yield from checkpin(self.tm.sh.poly_en, value=0)
                yield from self.tm.checkreply(self.tm.sh.COMMANDS.STOP, self.tm.state(state=states[idx]))
            elif test_command == self.tm.sh.COMMANDS.LASERTEST:
                yield from checkpin(self.tm.sh.poly_en, value=1)
                yield from checkpin(self.tm.sh.laser0, value=1)
                yield from self.tm.checkreply(self.tm.sh.COMMANDS.STOP, self.tm.state(state=states[idx]))
            elif test_command == self.tm.sh.COMMANDS.LINETEST:
                yield from checkpin(self.tm.sh.poly_en, value=0)
                yield from checkpin(self.tm.sh.laser0, value=1)
                yield from self.tm.checkreply(self.tm.sh.COMMANDS.STOP, self.tm.state(state=states[idx]))
            else:
                for _ in range(self.tm.sh.ticksinfacet): yield
                yield from self.tm.checkreply(self.tm.sh.COMMANDS.STOP, self.tm.state(state=self.tm.sh.STATES.STOP))

    @_test_decorator()
    def test_writedata(self):
        for i in range(self.tm.sh.MEMDEPTH+1):
            data_byte = i%256 # bytes can't be larger than 255
            if i%(self.tm.sh.CHUNKSIZE)==0:
                if (i>0)&((i%self.tm.sh.MEMDEPTH)==0):
                    # check if memory is full
                    yield from self.tm.checkreply(self.tm.sh.COMMANDS.WRITE_L, 
                                                self.tm.state(errors = [self.tm.sh.ERRORS.MEMFULL],
                                                    state = self.tm.sh.STATES.STOP))
                    continue
                else:
                    yield from self.tm.checkreply(self.tm.sh.COMMANDS.WRITE_L,
                                                self.tm.state(state=self.tm.sh.STATES.STOP))
            yield from self.tm.checkreply(data_byte, self.tm.state(state=self.tm.sh.STATES.STOP))
        in_memory = []
        loops = self.tm.sh.MEMDEPTH
        for i in range(loops):
            value = (yield self.tm.sh.mem[i])
            in_memory.append(value)
        self.assertEqual(list(range(loops)), in_memory)

    @_test_decorator()
    def test_nosync(self):
        yield self.tm.sh.photodiode.eq(1)
        yield from self.tm.checkreply(self.tm.sh.COMMANDS.STATUS, self.tm.state(state=self.tm.sh.STATES.STOP))
        yield from self.tm.checkreply(self.tm.sh.COMMANDS.START, self.tm.state(state=self.tm.sh.STATES.STOP))
        yield from checkenterstate(self.tm.sh.laserfsm, 'SPINUP')
        yield from checkenterstate(self.tm.sh.laserfsm, 'STATE_WAIT_STABLE')
        yield from checkpin(self.tm.sh.laser0, value = 1)
        yield from checkenterstate(self.tm.sh.laserfsm, 'STOP')
        yield from self.tm.checkreply(self.tm.sh.COMMANDS.STATUS,
                                    self.tm.state(errors=[self.tm.sh.ERRORS.NOTSTABLE, self.tm.sh.ERRORS.MEMREAD],
                                          state=self.tm.sh.STATES.STOP))

    @_test_decorator(simulatediode=True)
    def test_scanlinewithoutwrite(self):
        yield from self.tm.checkreply(self.tm.sh.COMMANDS.START, self.tm.state(state=self.tm.sh.STATES.STOP))
        yield from checkenterstate(self.tm.sh.laserfsm, 'STATE_WAIT_STABLE')
        yield from checkenterstate(self.tm.sh.laserfsm, 'READ_INSTRUCTION')
        yield from checkenterstate(self.tm.sh.laserfsm, 'WAIT_END')
        yield from checkenterstate(self.tm.sh.laserfsm, 'STATE_WAIT_STABLE')
        yield from self.tm.checkreply(self.tm.sh.COMMANDS.STOP, self.tm.state(errors=[self.tm.sh.ERRORS.MEMREAD],
                                                         state=self.tm.sh.STATES.START))

    @_test_decorator()
    def test_invalidspicommand(self):
        yield from self.tm.checkreply(255, self.tm.state(state=self.tm.sh.STATES.STOP))
        yield from self.tm.checkreply(self.tm.sh.COMMANDS.STATUS,
                                    self.tm.state(errors=[self.tm.sh.ERRORS.INVALID],
                                          state=self.tm.sh.STATES.STOP))

    @_test_decorator(simulatediode=True)
    def test_invalidscanline(self):
        '''check error received if scanline is sent with invalid command byte'''
        yield from self.tm.checkreply(self.tm.sh.COMMANDS.WRITE_L, self.tm.state(state=self.tm.sh.STATES.STOP))
        for _ in range(self.tm.sh.CHUNKSIZE):
            yield from self.tm.checkreply(int('11111101', 2), self.tm.state(state=self.tm.sh.STATES.STOP))
        yield from self.tm.checkreply(self.tm.sh.COMMANDS.START, self.tm.state(state=self.tm.sh.STATES.STOP))
        yield from checkenterstate(self.tm.sh.laserfsm, 'STATE_WAIT_STABLE')
        yield from checkenterstate(self.tm.sh.laserfsm, 'READ_INSTRUCTION')
        yield from checkenterstate(self.tm.sh.laserfsm, 'STOP')
        yield from self.tm.checkreply(self.tm.sh.COMMANDS.STOP, self.tm.state(errors=[self.tm.sh.ERRORS.INVALIDLINE],
                                                                              state=self.tm.sh.STATES.STOP))

    @_test_decorator(simulatediode=True)
    def test_stopscanline(self):
        '''check machine transitions to stop if stop command byte is sent'''
        yield from self.tm.writeline([])
        yield from self.tm.checkreply(self.tm.sh.COMMANDS.START, self.tm.state(state=self.tm.sh.STATES.STOP))
        yield from checkenterstate(self.tm.sh.laserfsm, 'STATE_WAIT_STABLE')
        yield from checkenterstate(self.tm.sh.laserfsm, 'READ_INSTRUCTION')
        yield from checkenterstate(self.tm.sh.laserfsm, 'STOP')
        yield from self.tm.checkreply(self.tm.sh.COMMANDS.STATUS, self.tm.state(state=self.tm.sh.STATES.STOP))

    @_test_decorator(singleline=True, simulatediode=True)
    def test_scanlinerepeated(self):
        '''test scanline with write in single line mode
        
        In this mode the line is always repeated.
        '''
        bitlst = [1,0]
        yield from self.tm.writeline(bitlst)
        yield from self.tm.checkreply(self.tm.sh.COMMANDS.START, self.tm.state(state=self.tm.sh.STATES.STOP))
        yield from self.tm.checkline(bitlst)
        bitlst = [1,1]
        yield from self.tm.writeline(bitlst)
        yield from self.tm.checkline(bitlst)

    @_test_decorator(singleline=True, singlefacet=True, simulatediode=True)
    def test_scanlinerepeatedsinglefacet(self):
        '''test scanline with write in single line and single facet mode
        
        Scanline is always repeated and only a single facet is used
        '''
        bitlst = [1,0]
        yield from self.tm.writeline(bitlst)
        yield from self.tm.checkreply(self.tm.sh.COMMANDS.START, self.tm.state(state=self.tm.sh.STATES.STOP))
        # facet counter changes
        for facet in range(self.tm.sh.VARIABLES['FACETS']-1):
            yield from checkenterstate(self.tm.sh.laserfsm, 'STATE_WAIT_STABLE')
            yield from checkenterstate(self.tm.sh.laserfsm, 'WAIT_END')
            self.assertEqual(facet, (yield self.tm.sh.facetcnt))
        # still line only projected at specific facet count
        for _ in range(2):
            yield from self.tm.checkline(bitlst)
            self.assertEqual(1, (yield self.tm.sh.facetcnt))
    
    @_test_decorator()
    def test_writespeed(self):
        '''test speed to write 1 byte to memory and give estimate for max laser speed
        '''
        def test_speed(count, byte):
            yield self.tm.spi.spimaster.mosi.eq(byte)
            yield self.tm.spi.spimaster.length.eq(8)
            yield self.tm.spi.spimaster.start.eq(1)
            count += 1
            yield
            yield self.tm.spi.spimaster.start.eq(0)
            count += 1
            yield
            while (yield self.tm.spi.spimaster.done) == 0:
                count += 1 
                yield
            byte_received = (yield self.tm.spi.spimaster.miso)
            return count, byte_received
        count = 0
        count, bytereceived = (yield from test_speed(count, self.tm.sh.COMMANDS.WRITE_L))
        self.assertEqual(bytereceived, self.tm.state(state=self.tm.sh.STATES.STOP))
        count, bytereceived = (yield from test_speed(count, 255))
        self.assertEqual(bytereceived, self.tm.state(state=self.tm.sh.STATES.STOP))
        def test_enter_state_speed(count, state, fsm = self.tm.sh.receiver):
            delta = 0
            while (fsm.decoding[(yield fsm.state)] != state):
                delta += 1
                if delta>100:
                    raise Exception(f"Reached {fsm.decoding[(yield fsm.state)]} not {state}")
                yield
            return count+delta
        count = (yield from test_enter_state_speed(count, 'WRITE'))
        count = (yield from test_enter_state_speed(count, 'IDLE'))
        #print(f"Writing 1 byte to memory takes {count} ticks")
        # NOTE: command byte is ignored
        #       it might be that one byte requires 3 transactions as memory is full
        #       part of circuit operates at 100E6 and part of 50E6 Hz
        #crystal = 50E6
        #print(f"Speed of laser must be lower than  {(crystal*8)/(count*1E6):.2f} Mhz")

    @_test_decorator(simulatediode=True)
    def test_scanlineringbuffer(self):
        '''test scanline with write using ring buffer
        '''
        for i in range(3):
            lines = [[1,0], [1,1], [0,1]] 
            for line in lines: yield from self.tm.writeline(line)
            if not i:
                yield from self.tm.checkreply(self.tm.sh.COMMANDS.START, self.tm.state(state=self.tm.sh.STATES.STOP))
            else:
                yield from self.tm.checkreply(self.tm.sh.COMMANDS.START, self.tm.state(state=self.tm.sh.STATES.STOP,
                                                                                errors=[self.tm.sh.ERRORS.MEMREAD]))
            for line in lines: yield from self.tm.checkline(line)
            yield from self.tm.checkreply(self.tm.sh.COMMANDS.STOP, self.tm.state(state=self.tm.sh.STATES.START,
                                                                                errors=[self.tm.sh.ERRORS.MEMREAD]))
            yield from checkenterstate(self.tm.sh.laserfsm, 'STOP')

if __name__ == '__main__':
    unittest.main()