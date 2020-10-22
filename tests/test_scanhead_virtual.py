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
        bytelst.reverse()
        for _ in range(math.ceil(len(bytelst)/sh.CHUNKSIZE)):
            state = self.get_state((yield from spi.xfer([sh.COMMANDS.WRITE_L]))[0])
            assert state['statebits'] in [sh.STATES.STOP, sh.STATES.START]
            if state['errorbits'] == pow(2, sh.ERRORS.MEMFULL): return bytelst
            for _ in range(sh.CHUNKSIZE): 
                try:
                    state = (yield from spi.xfer([bytelst.pop()]))
                except IndexError:
                    yield from spi.xfer([0])
        return bytelst

TM = TestMachine()
SH = TM.sh
STATE = TM.state

class TestScanhead(unittest.TestCase):
    ''' Virtual test for scanhead'''

    def _test_decorator(func):
        return lambda self: run_simulation(SH, [func(self)], clocks=TM.clocks)

    @_test_decorator
    def test_pwmgeneration(self):
        ''' verify generation of polygon pulse
        '''
        yield from checkpin(SH.poly_pwm, value=0)
        yield from checkpin(SH.poly_pwm, value=1)

    @_test_decorator
    def test_testmodes(self):
        ''' verify four test modes; laser, motor, line and photodiode
        '''
        test_commands = [SH.COMMANDS.LASERTEST,
                            SH.COMMANDS.MOTORTEST,
                            SH.COMMANDS.LINETEST,
                            SH.COMMANDS.PHOTODIODETEST]
        states = [SH.STATES.LASERTEST,
                    SH.STATES.MOTORTEST,
                    SH.STATES.LINETEST,
                    SH.STATES.PHOTODIODETEST]
        # NOTE: on default photodiode is high
        yield SH.photodiode.eq(1) 
        for idx, test_command in enumerate(test_commands):
            yield from TM.checkreply(SH.COMMANDS.STATUS, STATE(state=SH.STATES.STOP))
            yield from TM.checkreply(test_command, STATE(state=SH.STATES.STOP))
            if test_command != SH.COMMANDS.MOTORTEST:
                yield from checkpin(SH.laser0)
            if test_command != SH.COMMANDS.LASERTEST:
                yield from checkpin(SH.poly_en, value=1)
            if test_command == SH.COMMANDS.PHOTODIODETEST:
                for _ in range(6): yield
                yield SH.photodiode.eq(0) 
                yield from checkpin(SH.laser0, 1)
                yield from checkpin(SH.poly_en, value=0)
                yield from TM.checkreply(SH.COMMANDS.STOP, STATE(state=SH.STATES.STOP))
            else:
                yield from TM.checkreply(SH.COMMANDS.STOP, STATE(state=states[idx]))

    @_test_decorator
    def test_writedata(self):
        for i in range(SH.MEMDEPTH+1):
            data_byte = i%256 # bytes can't be larger than 255
            if i%(SH.CHUNKSIZE)==0:
                if (i>0)&((i%SH.MEMDEPTH)==0):
                    # check if memory is full
                    yield from TM.checkreply(SH.COMMANDS.WRITE_L, 
                                                STATE(errors = [SH.ERRORS.MEMFULL],
                                                    state = SH.STATES.STOP))
                    continue
                else:
                    yield from TM.checkreply(SH.COMMANDS.WRITE_L,
                                                STATE(state=SH.STATES.STOP))
            yield from TM.checkreply(data_byte, STATE(state=SH.STATES.STOP))
        in_memory = []
        loops = SH.MEMDEPTH
        for i in range(loops):
            value = (yield SH.mem[i])
            in_memory.append(value)
        self.assertEqual(list(range(loops)), in_memory)

    @_test_decorator
    def test_nosync(self):
        yield SH.photodiode.eq(1) 
        yield from TM.checkreply(SH.COMMANDS.STATUS, STATE(state=SH.STATES.STOP))
        yield from TM.checkreply(SH.COMMANDS.START, STATE(state=SH.STATES.STOP))
        yield from checkenterstate(SH.laserfsm, 'SPINUP')
        yield from checkenterstate(SH.laserfsm, 'STATE_WAIT_STABLE')
        yield from checkpin(SH.laser0, value = 1)
        yield from checkenterstate(SH.laserfsm, 'STOP')
        yield from TM.checkreply(SH.COMMANDS.STATUS,
                                    STATE(errors=[SH.ERRORS.NOTSTABLE],
                                          state=SH.STATES.STOP))

    @_test_decorator
    def test_scanlinewithoutwrite(self):
        yield SH.photodiode.eq(1) 
        yield from TM.checkreply(SH.COMMANDS.START, STATE(state=SH.STATES.STOP))
        yield from checkenterstate(SH.laserfsm, 'STATE_WAIT_STABLE')
        yield from TM.photodiode_trigger()
        yield from checkenterstate(SH.laserfsm, 'READ_INSTRUCTION')
        yield from checkenterstate(SH.laserfsm, 'WAIT_END')
        yield from checkenterstate(SH.laserfsm, 'STATE_WAIT_STABLE')
        yield from TM.photodiode_trigger()
        yield from TM.checkreply(SH.COMMANDS.STOP, STATE(errors=[SH.ERRORS.MEMREAD],
                                                         state=SH.STATES.START))

    @_test_decorator
    def test_invalidspicommand(self):
        yield from TM.checkreply(255, STATE(state=SH.STATES.STOP))
        yield from TM.checkreply(SH.COMMANDS.STATUS,
                                    STATE(errors=[SH.ERRORS.INVALID],
                                          state=SH.STATES.STOP))

    @_test_decorator
    def test_invalidscanline(self):
        ''' check error received if scanline is sent with invalid command byte'''
        yield from TM.checkreply(SH.COMMANDS.WRITE_L, STATE(state=SH.STATES.STOP))
        for _ in range(SH.CHUNKSIZE):
            yield from TM.checkreply(int('11111101', 2), STATE(state=SH.STATES.STOP))
        yield SH.photodiode.eq(1)
        yield from TM.checkreply(SH.COMMANDS.START, STATE(state=SH.STATES.STOP))
        yield from checkenterstate(SH.laserfsm, 'STATE_WAIT_STABLE')
        yield from TM.photodiode_trigger()
        yield from checkenterstate(SH.laserfsm, 'WAIT_END')
        yield from TM.checkreply(SH.COMMANDS.STOP, STATE(errors=[SH.ERRORS.INVALIDLINE],
                                            state=SH.STATES.START))
        yield from checkenterstate(SH.laserfsm, 'STOP')

    @_test_decorator
    def test_stopscanline(self):
        ''' check machine transitions to stop if stop command byte is sent
        '''
        yield from TM.writeline([0]*SH.BITSINSCANLINE, lastline=True)
        yield from TM.checkreply(SH.COMMANDS.START, STATE(state=SH.STATES.STOP))
        yield SH.photodiode.eq(1)
        yield from checkenterstate(SH.laserfsm, 'STATE_WAIT_STABLE')
        yield from TM.photodiode_trigger()
        yield from checkenterstate(SH.laserfsm, 'READ_INSTRUCTION')
        yield from checkenterstate(SH.laserfsm, 'STOP')
        yield from TM.checkreply(SH.COMMANDS.STATUS, STATE(state=SH.STATES.STOP))

    # def test_scanlinewithwrite(self):
    #     def cpu_side():
    #         for _ in range(int(Scanhead.MEMDEPTH/Scanhead.MEMWIDTH)): 
    #             yield from self.checkreply(Scanhead.COMMANDS.WRITE_L, self.state(STATE=Scanhead.STATES.STOP))
    #             for _ in range(Scanhead.CHUNKSIZE): yield from self.checkreply(int('11111101', 2), self.state(STATE=Scanhead.STATES.STOP))
    #         # check if mem is full
    #         yield from self.checkreply(Scanhead.COMMANDS.STATUS, self.state(errors=[Scanhead.ERRORS.MEMFULL],
    #                                                                          STATE=Scanhead.STATES.STOP))
    #         # turn on laser head
    #         yield self.sh.photodiode.eq(1) 
    #         yield from self.checkreply(Scanhead.COMMANDS.START, self.state(errors=[Scanhead.ERRORS.MEMFULL],
    #                                                                          STATE=Scanhead.STATES.STOP))
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
    #     run_simulation(self.scanhead, [cpu_side()])

if __name__ == '__main__':
    unittest.main()
