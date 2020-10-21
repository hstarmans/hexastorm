import unittest
import math

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
    while (yield pin) == value:
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
            raise Exception(f"State not reached")
        yield

class TestMachine():
    ''' helper class for the test machine '''
    def __init__(self):
        sh = Scanhead
        self.clocks = {"sys": 20, 'clk100':10}
        self.ticksinfacet = 12
        self.laserticks = 4
        sh.MEMDEPTH = 8
        sh.CHUNKSIZE = 2
        sh.VARIABLES['CRYSTAL_HZ']= round(self.ticksinfacet*sh.VARIABLES['FACETS']
                                          *sh.VARIABLES['RPM']/60)
        sh.VARIABLES['LASER_HZ'] = sh.VARIABLES['CRYSTAL_HZ']/self.laserticks
        sh.VARIABLES['SPINUP_TIME'] = 10/sh.VARIABLES['CRYSTAL_HZ']
        sh.VARIABLES['STABLE_TIME'] = 30/sh.VARIABLES['CRYSTAL_HZ']
        sh.VARIABLES['START%'] = 2/self.ticksinfacet
        scanbits = 2
        Scanhead.VARIABLES['END%'] = (self.laserticks*scanbits)/self.ticksinfacet + sh.VARIABLES['START%']
        self.mach = Machine(connected=False)
        self.mach.sh = self.sh = Scanhead(board.Platform(), test=True)
        self.sh.submodules.master = SPIMaster(self.sh.spi, data_width=8, sys_clk_freq=100e6,
                                              spi_clk_freq=5e6, with_csr=False)
        self.mach.spi = FakeSpi(self.sh.master)
    
    def checkreply(self, command, data_expected):
        '''command is sent and checked against result
        '''
        data_received = (yield from self.mach.spi.xfer([command]))[0]
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

    def photodiode_trigger(self):
        '''triggers photodiode at end facet
        '''
        sh = self.sh
        count = (yield sh.tickcounter)
        #TODO: remove jitter ticks, should always substract 3!
        for _ in range(sh.ticksinfacet-sh.JITTERTICKS-1-count): yield
        yield sh.photodiode.eq(0)
        yield
        yield
        yield sh.photodiode.eq(1)
        yield

    def get_state(self, byte):
        '''
        places state and error bits in dict
        '''
        return {'statebits': byte>>5, 'errorbits': byte&0b11111}

    def writeline(self, bytelst, lastline = False):
        '''
        writes bytelst to memory
            if bytelst is empty --> last line command is given
            preceded by scan command

        return: the bytes it wasn't able to write if memory gets full
        '''
        sh, spi = self.sh, self.mach.spi
        assert len(bytelst) == sh.BITSINSCANLINE
        assert max(bytelst) <= 255
        assert min(bytelst) >= 0
        bytelst = [sh.INSTRUCTIONS.STOP] + bytelst if lastline else [sh.INSTRUCTIONS.SCAN] + bytelst
        for _ in range(math.ceil(len(bytelst)/sh.MEMWIDTH)):
            state = self.get_state((yield from spi.xfer([sh.COMMANDS.WRITE_L]))[0])
            assert state['statebits'] in [sh.STATES.STOP, sh.STATES.START]
            if state['errorbits'] == pow(2, sh.ERRORS.MEMFULL): return bytelst
            for _ in range(sh.CHUNKSIZE): 
                try:
                    state = (yield from spi.xfer([bytelst.pop()]))
                except IndexError:
                    yield from spi.xfer([0])
        return bytelst

class TestScanhead(unittest.TestCase):
    ''' Virtual test for scanhead'''

    def setUp(self):
        self.tm = TestMachine()
        self.clocks = self.tm.clocks

    def test_pwmgeneration(self):
        ''' verify generation of polygon pulse
        '''
        sh = self.tm.sh
        def cpu_side():
            yield from checkpin(sh.poly_pwm, value=0)
            yield from checkpin(sh.poly_pwm, value=1)
        run_simulation(sh, [cpu_side()], clocks=self.clocks)

    def test_testmodes(self):
        ''' verify four test modes; laser, motor, line and photodiode
        '''
        tm = self.tm
        state, sh = tm.state, tm.sh
        def cpu_side():
            test_commands = [sh.COMMANDS.LASERTEST,
                             sh.COMMANDS.MOTORTEST,
                             sh.COMMANDS.LINETEST,
                             sh.COMMANDS.PHOTODIODETEST]
            states = [sh.STATES.LASERTEST,
                      sh.STATES.MOTORTEST,
                      sh.STATES.LINETEST,
                      sh.STATES.PHOTODIODETEST]
            # NOTE: on default photodiode is high
            yield sh.photodiode.eq(1) 
            for idx, test_command in enumerate(test_commands):
                yield from tm.checkreply(sh.COMMANDS.STATUS, state(state=sh.STATES.STOP))
                yield from tm.checkreply(test_command, state(state=sh.STATES.STOP))
                if test_command != sh.COMMANDS.MOTORTEST:
                    yield from checkpin(sh.laser0)
                if test_command != sh.COMMANDS.LASERTEST:
                    yield from checkpin(sh.poly_en, value=1)
                if test_command == sh.COMMANDS.PHOTODIODETEST:
                    yield sh.photodiode.eq(0) 
                    # check if laser and polygon turned off
                    yield from checkpin(sh.laser0, 1)
                    yield from checkpin(sh.poly_en, value=0)
                    yield from tm.checkreply(sh.COMMANDS.STOP, state(state=sh.STATES.STOP))
                else:
                    yield from tm.checkreply(sh.COMMANDS.STOP, state(state=states[idx]))
        #run_simulation(sh, [cpu_side()], clocks=self.clocks)

    def test_writedata(self):
        tm = self.tm
        state, sh = tm.state, tm.sh
        def cpu_side():
            for i in range(sh.MEMDEPTH+1):
                data_byte = i%256 # bytes can't be larger than 255
                if i%(sh.CHUNKSIZE)==0:
                    if (i>0)&((i%sh.MEMDEPTH)==0):
                        # check if memory is full
                        yield from tm.checkreply(sh.COMMANDS.WRITE_L, 
                                                  state(errors = [sh.ERRORS.MEMFULL],
                                                        state = sh.STATES.STOP))
                        continue
                    else:
                        yield from tm.checkreply(sh.COMMANDS.WRITE_L,
                                                   state(state=sh.STATES.STOP))
                yield from tm.checkreply(data_byte, state(state=sh.STATES.STOP))
            in_memory = []
            loops = sh.MEMDEPTH
            for i in range(loops):
                value = (yield sh.mem[i])
                in_memory.append(value)
            self.assertEqual(list(range(loops)),in_memory)
        #run_simulation(sh, [cpu_side()])

    def test_nosync(self):
        tm = self.tm
        state, sh = tm.state, tm.sh
        def cpu_side():
            yield sh.photodiode.eq(1) 
            yield from tm.checkreply(sh.COMMANDS.STATUS, state(state=sh.STATES.STOP))
            yield from tm.checkreply(sh.COMMANDS.START, state(state=sh.STATES.STOP))
            yield from checkenterstate(sh.laserfsm, 'SPINUP')
            yield from checkenterstate(sh.laserfsm, 'STATE_WAIT_STABLE')
            yield from checkpin(sh.laser0, value = 1)
            yield from checkenterstate(sh.laserfsm, 'STOP')
            yield from tm.checkreply(sh.COMMANDS.STATUS,
                                     state(errors=[sh.ERRORS.NOTSTABLE],
                                           state=sh.STATES.STOP))
        #run_simulation(sh, [cpu_side()])

    def test_scanlinewithoutwrite(self):
        tm = self.tm
        state, sh = tm.state, tm.sh
        def cpu_side():
            yield sh.photodiode.eq(1) 
            yield from tm.checkreply(sh.COMMANDS.START, state(state=sh.STATES.STOP))
            yield from checkenterstate(sh.laserfsm, 'STATE_WAIT_STABLE')
            yield from tm.photodiode_trigger()
            yield from checkenterstate(sh.laserfsm, 'READ_INSTRUCTION')
            yield from checkenterstate(sh.laserfsm, 'WAIT_END')
            yield from checkenterstate(sh.laserfsm, 'STATE_WAIT_STABLE')
            yield from tm.photodiode_trigger()
            yield from tm.checkreply(sh.COMMANDS.STOP, state(errors=[sh.ERRORS.MEMREAD],
                                                                state=sh.STATES.START))
        #run_simulation(sh, [cpu_side()])

    def test_invalidspicommand(self):
        tm = self.tm
        state, sh = tm.state, tm.sh
        def cpu_side():
            yield from tm.checkreply(255, state(state=sh.STATES.STOP))
            yield from tm.checkreply(sh.COMMANDS.STATUS,
                                      state(errors=[sh.ERRORS.INVALID],
                                            state=sh.STATES.STOP))
        #run_simulation(sh, [cpu_side()])

    def test_invalidscanline(self):
        ''' check error received if scanline is sent with invalid command byte'''
        tm = self.tm
        state, sh = tm.state, tm.sh
        def cpu_side():
            yield from tm.checkreply(sh.COMMANDS.WRITE_L, state(state=sh.STATES.STOP))
            for _ in range(sh.CHUNKSIZE):
                yield from tm.checkreply(int('11111101', 2), state(state=sh.STATES.STOP))
            yield sh.photodiode.eq(1)
            yield from tm.checkreply(sh.COMMANDS.START, state(state=sh.STATES.STOP))
            yield from checkenterstate(sh.laserfsm, 'STATE_WAIT_STABLE')
            yield from tm.photodiode_trigger()
            yield from checkenterstate(sh.laserfsm, 'WAIT_END')
            yield from tm.checkreply(sh.COMMANDS.STOP, state(errors=[sh.ERRORS.INVALIDLINE],
                                                state=sh.STATES.START))
            yield from checkenterstate(sh.laserfsm, 'STOP')
        #run_simulation(sh, [cpu_side()])

    def test_stopscanline(self):
        ''' check machine transitions to stop if stop command byte is sent
        '''
        tm = self.tm
        state, sh = tm.state, tm.sh
        def cpu_side():
            yield from tm.writeline([0]*sh.BITSINSCANLINE, lastline=True)
            yield from tm.checkreply(sh.COMMANDS.START, state(state=sh.STATES.STOP))
            yield sh.photodiode.eq(1)
            yield from checkenterstate(sh.laserfsm, 'STATE_WAIT_STABLE')
            yield from tm.photodiode_trigger()
            yield from checkenterstate(sh.laserfsm, 'WAIT_END')
            yield from tm.checkreply(sh.COMMANDS.STOP, state(errors=[sh.ERRORS.INVALIDLINE],
                                                             state=sh.STATES.START))
            yield from checkenterstate(sh.laserfsm, 'STOP')
        run_simulation(sh, [cpu_side()])

    # def test_scanlinewithwrite(self):
    #     def cpu_side():
    #         for _ in range(int(Scanhead.MEMDEPTH/Scanhead.MEMWIDTH)): 
    #             yield from self.checkreply(Scanhead.COMMANDS.WRITE_L, self.state(state=Scanhead.STATES.STOP))
    #             for _ in range(Scanhead.CHUNKSIZE): yield from self.checkreply(int('11111101', 2), self.state(state=Scanhead.STATES.STOP))
    #         # check if mem is full
    #         yield from self.checkreply(Scanhead.COMMANDS.STATUS, self.state(errors=[Scanhead.ERRORS.MEMFULL],
    #                                                                          state=Scanhead.STATES.STOP))
    #         # turn on laser head
    #         yield self.sh.photodiode.eq(1) 
    #         yield from self.checkreply(Scanhead.COMMANDS.START, self.state(errors=[Scanhead.ERRORS.MEMFULL],
    #                                                                          state=Scanhead.STATES.STOP))
    #         # check if statemachine goes to spinup state
    #         yield from self.checkenterstate(self.sh.laserfsm, 'SPINUP')
    #         # check if statemachine goes to statewaitstable
    #         yield from self.checkenterstate(self.sh.laserfsm, 'STATE_WAIT_STABLE')
    #         # finish cycle
    #         for _ in range(1): yield from self.photodiode_trigger()
    #         # counter should be clean before as it enters "WAIT_FOR_DATA_RUN"
    #         self.assertEqual((yield self.sh.tickcounter), 0)
    #         self.assertEqual((yield from self.getstate(self.sh.laserfsm)), 'WAIT_FOR_DATA_RUN')
    #         startwaitticks = int(Scanhead.VARIABLES['START%']*self.ticksinfacet)
    #         for _ in range(startwaitticks-1): yield
    #         self.assertEqual((yield self.sh.tickcounter), startwaitticks-1)
    #         self.assertEqual((yield from self.getstate(self.sh.laserfsm)), 'DATA_RUN')
    #         self.assertEqual((yield self.sh.laser0), 0)
    #         self.assertEqual((yield self.sh.scanbit),0)
    #         self.assertEqual((yield self.sh.lasercnt), 0)
    #         # You need one tick to start reading first pixel
    #         yield
    #         # Pixel 0
    #         self.assertEqual((yield self.sh.tickcounter), startwaitticks)
    #         self.assertEqual((yield self.sh.laser0), 1)
    #         self.assertEqual((yield self.sh.lasercnt), 1)
    #         yield
    #         self.assertEqual((yield self.sh.laser0), 1)
    #         self.assertEqual((yield self.sh.scanbit),1)
    #         self.assertEqual((yield self.sh.lasercnt), 0)
    #         yield
    #         # Pixel 1
    #         self.assertEqual((yield self.sh.scanbit),2)
    #         self.assertEqual((yield self.sh.lasercnt), 1)
    #         self.assertEqual((yield from self.getstate(self.sh.laserfsm)), 'DATA_RUN')
    #         self.assertEqual((yield self.sh.tickcounter), startwaitticks + self.laserticks)
    #         self.assertEqual((yield self.sh.laser0), 0)
    #         yield
    #         self.assertEqual((yield self.sh.lasercnt), 0)
    #         self.assertEqual((yield self.sh.laser0), 0)
    #         yield
    #         self.assertEqual((yield self.sh.lasercnt), 1)
    #         self.assertEqual((yield from self.getstate(self.sh.laserfsm)), 'WAIT_END')
    #         self.assertEqual((yield self.sh.tickcounter), int(Scanhead.VARIABLES['END%']*self.ticksinfacet))
    #         self.assertEqual((yield self.sh.laser0), 0)
    #         # check wether it returns to wait state stable
    #         yield from self.checkenterstate(self.sh.laserfsm, 'STATE_WAIT_STABLE')
    #         self.assertEqual((yield self.sh.tickcounter), int(self.ticksinfacet*(1-Scanhead.VARIABLES['SYNCSTART'])))
    #         yield
    #         self.assertEqual((yield self.sh.laser0), 1)
    #     #run_simulation(self.scanhead, [cpu_side()])

if __name__ == '__main__':
    unittest.main()
