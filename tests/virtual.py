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

def checkpin(pin, value=0, maxticks=100):
    'do maxticks until pin reaches value'
    ticks = 0
    while (yield pin) != value:
        ticks += 1
        if ticks>maxticks:
            raise Exception(f"Pin doesnt reach state in {maxticks}")
        yield

def getstate(fsm):
    'retrieves encoding of state of finite state machine'
    return fsm.decoding[(yield fsm.state)]

def checkenterstate(fsm, wait_state, maxticks=100):
    'checks if state is entered'
    ticks  = 0
    while (fsm.decoding[(yield fsm.state)] != wait_state):
        ticks += 1
        if ticks>maxticks:
            raise Exception(f"Reached {fsm.decoding[(yield fsm.state)]} not {wait_state}")
        yield

class TestMachine(Machine):
    ''' helper class for the test machine '''
    def __init__(self):
        sh = Scanhead
        self.clocks = {"sys": 20, 'clk100':10}
        self.ticksinfacet = 18
        self.laserticks = 4
        #TODO: MEMDEPTH is actually overwritten, make a decision here
        sh.MEMDEPTH = 8
        sh.CHUNKSIZE = 1
        sh.VARIABLES['CRYSTAL_HZ']= round(self.ticksinfacet*sh.VARIABLES['FACETS']
                                          *sh.VARIABLES['RPM']/60)
        sh.VARIABLES['LASER_HZ'] = sh.VARIABLES['CRYSTAL_HZ']/self.laserticks
        sh.VARIABLES['SPINUP_TIME'] = 10/sh.VARIABLES['CRYSTAL_HZ']
        sh.VARIABLES['STABLE_TIME'] = 50/sh.VARIABLES['CRYSTAL_HZ']  #TODO: stop scanline seems to affect the stable thresh?! can be 30 without stopline
        sh.VARIABLES['START%'] = 2/self.ticksinfacet
        scanbits = 2
        Scanhead.VARIABLES['END%'] = (self.laserticks*scanbits)/self.ticksinfacet + sh.VARIABLES['START%']
        self.flash()
        
    def flash(self):
        self.sh = Scanhead(board.Platform(), test=True)
        self.sh.submodules.master = SPIMaster(self.sh.spi, data_width=8, sys_clk_freq=100e6,
                                              spi_clk_freq=5e6, with_csr=False)
        self.spi = FakeSpi(self.sh.master)

    def checkreply(self, command, state=Scanhead.STATES.STOP, errors=[]):
        'command is sent and checked against expected byte'
        data_received = (yield from self.spi.xfer([command]))[0]
        try:
            assert data_received == self.statetobyte(errors=errors, state=state)
        except AssertionError:
            machine_state, error_string = self.status(byte=data_received, verbose=False)
            raise Exception(f"State {machine_state} and errors {error_string} not expected")

    def checkline(self, bitlst):
        'it is verified wether the laser produces the pattern in bitlist'
        yield from checkenterstate(self.sh.laserfsm, 'READ_INSTRUCTION')
        yield from checkenterstate(self.sh.laserfsm, 'DATA_RUN')
        yield
        for idx, bit in enumerate(bitlst):
            assert (yield self.sh.lasercnt) == self.laserticks-1
            assert (yield self.sh.scanbit) == idx+1
            for _ in range(self.laserticks):
                assert (yield self.sh.laser0) == bit
                yield

    def getmemory(self):
        'returns the SRAM memory as list'
        in_memory = []
        depth = self.sh.MEMDEPTH
        for i in range(depth):
            value = (yield self.sh.mem[i])
            in_memory.append(value)
        return in_memory

    def forcewrite(self, data, maxtrials=10):
        state = self.bytetostate((yield from self.spi.xfer([data]))[0])
        trials = 0
        while state['errorbits'][self.sh.ERRORS.MEMFULL]:
           state = self.bytetostate((yield from self.spi.xfer([data]))[0])
           trials += 1
           if trials>maxtrials:
                raise Exception(f"More than {maxtrials} required to write to memory")

    def writeline(self, bitlst):
        '''
        writes bitlist to memory
        if bitlst is empty --> stop command is sent
        '''
        bytelst = self.bittobytelist(bitlst)
        g = self.genwritebytes(bytelst)
        for byte in g: yield from self.forcewrite(byte)


class Tests(unittest.TestCase):
    ''' Virtual test for scanhead'''
    def setUp(self):
        self.tm = TestMachine()

    def _test_decorator(singleline=False, singlefacet=False, simulatediode=False):
        def nested_dec(func):
            def functie(self):
                if singlefacet or singleline:
                     self.tm.single_facet, self.tm.single_line = singlefacet, singleline
                     self.tm.flash()
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
        'verify generation of polygon pulse'
        yield from checkpin(self.tm.sh.poly_pwm, value=0)
        yield from checkpin(self.tm.sh.poly_pwm, value=1)

    @_test_decorator(simulatediode=True)
    def test_testmodes(self):
        'verify four test modes; laser, motor, line and photodiode'
        test_commands = [self.tm.sh.COMMANDS.LASERTEST,
                            self.tm.sh.COMMANDS.MOTORTEST,
                            self.tm.sh.COMMANDS.LINETEST,
                            self.tm.sh.COMMANDS.PHOTODIODETEST]
        states = [self.tm.sh.STATES.LASERTEST,
                    self.tm.sh.STATES.MOTORTEST,
                    self.tm.sh.STATES.LINETEST,
                    self.tm.sh.STATES.PHOTODIODETEST]
        for idx, test_command in enumerate(test_commands):
            yield from self.tm.checkreply(self.tm.sh.COMMANDS.STATUS, state=self.tm.sh.STATES.STOP)
            yield from self.tm.checkreply(test_command, state=self.tm.sh.STATES.STOP)
            if test_command == self.tm.sh.COMMANDS.MOTORTEST:
                yield from checkpin(self.tm.sh.laser0, value=0)
                yield from checkpin(self.tm.sh.poly_en, value=0)
                yield from self.tm.checkreply(self.tm.sh.COMMANDS.STOP, state=states[idx])
            elif test_command == self.tm.sh.COMMANDS.LASERTEST:
                yield from checkpin(self.tm.sh.poly_en, value=1)
                yield from checkpin(self.tm.sh.laser0, value=1)
                yield from self.tm.checkreply(self.tm.sh.COMMANDS.STOP, state=states[idx])
            elif test_command == self.tm.sh.COMMANDS.LINETEST:
                yield from checkpin(self.tm.sh.poly_en, value=0)
                yield from checkpin(self.tm.sh.laser0, value=1)
                yield from self.tm.checkreply(self.tm.sh.COMMANDS.STOP, state=states[idx])
            else:
                for _ in range(self.tm.sh.ticksinfacet): yield
                yield from self.tm.checkreply(self.tm.sh.COMMANDS.STOP, state=self.tm.sh.STATES.STOP)

    @_test_decorator()
    def test_memory(self):
        #TODO: memdepth is overwritten
        depth = self.tm.sh.MEMDEPTH-self.tm.sh.bytesinline
        for i in range(depth//Scanhead.CHUNKSIZE):
            statebyte = (yield from self.tm.spi.xfer([Scanhead.COMMANDS.WRITE_L]))[0]
            state = self.tm.bytetostate(statebyte)
            self.assertEqual(state['errorbits'][self.tm.sh.ERRORS.MEMFULL], 0)
            for _ in range(Scanhead.CHUNKSIZE):
                yield from self.tm.spi.xfer([i])
        yield from self.tm.checkreply(self.tm.sh.COMMANDS.STATUS, state=self.tm.sh.STATES.STOP, errors=[self.tm.sh.ERRORS.MEMFULL])
        self.assertEqual(list(range(depth)), self.tm.getmemory())

    @_test_decorator()
    def test_nosync(self):
        yield self.tm.sh.photodiode.eq(1)
        yield from self.tm.checkreply(self.tm.sh.COMMANDS.STATUS, state=self.tm.sh.STATES.STOP)
        yield from self.tm.checkreply(self.tm.sh.COMMANDS.START, state=self.tm.sh.STATES.STOP)
        yield from checkenterstate(self.tm.sh.laserfsm, 'SPINUP')
        yield from checkenterstate(self.tm.sh.laserfsm, 'STATE_WAIT_STABLE')
        yield from checkpin(self.tm.sh.laser0, value = 1)
        yield from checkenterstate(self.tm.sh.laserfsm, 'STOP')
        yield from self.tm.checkreply(self.tm.sh.COMMANDS.STATUS, errors=[self.tm.sh.ERRORS.NOTSTABLE, self.tm.sh.ERRORS.MEMREAD],
                                          state=self.tm.sh.STATES.STOP)

    @_test_decorator(simulatediode=True)
    def test_scanlinewithoutwrite(self):
        yield from self.tm.checkreply(self.tm.sh.COMMANDS.START, state=self.tm.sh.STATES.STOP)
        yield from checkenterstate(self.tm.sh.laserfsm, 'STATE_WAIT_STABLE')
        yield from checkenterstate(self.tm.sh.laserfsm, 'READ_INSTRUCTION')
        yield from checkenterstate(self.tm.sh.laserfsm, 'WAIT_END')
        yield from checkenterstate(self.tm.sh.laserfsm, 'STATE_WAIT_STABLE')
        yield from self.tm.checkreply(self.tm.sh.COMMANDS.STOP, errors=[self.tm.sh.ERRORS.MEMREAD],
                                                         state=self.tm.sh.STATES.START)

    @_test_decorator()
    def test_invalidspicommand(self):
        yield from self.tm.checkreply(255, state=self.tm.sh.STATES.STOP)
        yield from self.tm.checkreply(self.tm.sh.COMMANDS.STATUS, errors=[self.tm.sh.ERRORS.INVALID],
                                          state=self.tm.sh.STATES.STOP)

    @_test_decorator(simulatediode=True)
    def test_invalidscanline(self):
        'check error received if scanline is sent with invalid command byte'
        yield from self.tm.checkreply(self.tm.sh.COMMANDS.WRITE_L, state=self.tm.sh.STATES.STOP)
        for _ in range(self.tm.sh.CHUNKSIZE):
            yield from self.tm.checkreply(int('11111101', 2), state=self.tm.sh.STATES.STOP)
        yield from self.tm.checkreply(self.tm.sh.COMMANDS.START, state=self.tm.sh.STATES.STOP)
        yield from checkenterstate(self.tm.sh.laserfsm, 'STATE_WAIT_STABLE')
        yield from checkenterstate(self.tm.sh.laserfsm, 'READ_INSTRUCTION')
        yield from checkenterstate(self.tm.sh.laserfsm, 'STOP')
        yield from self.tm.checkreply(self.tm.sh.COMMANDS.STOP, errors=[self.tm.sh.ERRORS.INVALIDLINE],
                                                                              state=self.tm.sh.STATES.STOP)

    @_test_decorator(simulatediode=True)
    def test_stopscanline(self):
        'check machine transitions to stop if stop command byte is sent'
        yield from self.tm.writeline([])
        yield from self.tm.checkreply(self.tm.sh.COMMANDS.START, state=self.tm.sh.STATES.STOP)
        yield from checkenterstate(self.tm.sh.laserfsm, 'STATE_WAIT_STABLE')
        yield from checkenterstate(self.tm.sh.laserfsm, 'READ_INSTRUCTION')
        yield from checkenterstate(self.tm.sh.laserfsm, 'STOP')
        yield from self.tm.checkreply(self.tm.sh.COMMANDS.STATUS, state=self.tm.sh.STATES.STOP)

    @_test_decorator(singleline=True, simulatediode=True)
    def test_scanlinerepeated(self):
        '''test scanline with write in single line mode
        
        In this mode the line is always repeated.
        '''
        bitlst = [1,0]
        yield from self.tm.writeline(bitlst)
        yield from self.tm.checkreply(self.tm.sh.COMMANDS.START, state=self.tm.sh.STATES.STOP)
        yield from self.tm.checkline(bitlst)
        bitlst = [1,1]
        yield from self.tm.writeline(bitlst)
        # ensure write is received
        yield from checkenterstate(self.tm.sh.receiver, 'WRITE')
        yield from checkenterstate(self.tm.sh.receiver, 'IDLE')
        yield from self.tm.checkline(bitlst)

    @_test_decorator(singleline=True, singlefacet=True, simulatediode=True)
    def test_scanlinerepeatedsinglefacet(self):
        '''test scanline with write in single line and single facet mode
        
        Scanline is always repeated and only a single facet is used
        '''
        bitlst = [1,0]
        yield from self.tm.writeline(bitlst)
        yield from self.tm.checkreply(self.tm.sh.COMMANDS.START, state=self.tm.sh.STATES.STOP)
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
    def test_writespeed(self, verbose=False):
        'test speed to write 1 byte to memory and give estimate for max laser speed'
        #NOTE: this is not really a test, bit of experimental work
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
        self.assertEqual(bytereceived, self.tm.statetobyte(state=self.tm.sh.STATES.STOP))
        count, bytereceived = (yield from test_speed(count, 255))
        self.assertEqual(bytereceived, self.tm.statetobyte(state=self.tm.sh.STATES.STOP))
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
        if verbose:
            print(f"Writing 1 byte to memory takes {count} ticks")
            # NOTE: command byte is ignored
            #     it might be that one byte requires 3 transactions as memory is full
            #     part of circuit operates at 100E6 and part of 50E6 Hz
            crystal = 50E6
            print(f"Speed of laser must be lower than  {(crystal*8)/(count*1E6):.2f} Mhz")

    @_test_decorator(simulatediode=True)
    def test_scanlineringbuffer(self):
        'test scanline with write using ring buffer'
        lines = [[1,1], [1,1], [0,1], [0,0]]
        for line in lines: yield from self.tm.writeline(line)
        yield from self.tm.checkreply(self.tm.sh.COMMANDS.START, state=self.tm.sh.STATES.STOP, errors=[self.tm.sh.ERRORS.MEMFULL])
        for line in lines: yield from self.tm.checkline(line)
        for line in lines: yield from self.tm.writeline(line)
        for _ in range(self.tm.sh.ticksinfacet*len(lines)): yield
        yield from self.tm.checkreply(self.tm.sh.COMMANDS.STOP, state=self.tm.sh.STATES.START, errors=[self.tm.sh.ERRORS.MEMREAD])
        yield from checkenterstate(self.tm.sh.laserfsm, 'STOP')
        for _ in range(3):
            lines = [[1,0], [1,1], [0,1]]
            for line in lines: yield from self.tm.writeline(line)
            yield from self.tm.checkreply(self.tm.sh.COMMANDS.START, state=self.tm.sh.STATES.STOP,
                                                                     errors=[self.tm.sh.ERRORS.MEMREAD])
            for line in lines: yield from self.tm.checkline(line)
            yield from self.tm.checkreply(self.tm.sh.COMMANDS.STOP, state=self.tm.sh.STATES.START,
                                                                    errors=[self.tm.sh.ERRORS.MEMREAD])
            yield from checkenterstate(self.tm.sh.laserfsm, 'STOP')

if __name__ == '__main__':
    unittest.main()