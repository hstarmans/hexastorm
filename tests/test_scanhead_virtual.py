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


class TestScanhead(unittest.TestCase):
    ''' Virtual test for scanhead'''

    def setUp(self):
        self.clocks = {"sys": 20, 'clk100':10}
        self.ticksinfacet = 12
        self.laserticks = 4
        Scanhead.MEMDEPTH = 8
        Scanhead.VARIABLES['CRYSTAL_HZ']= round(self.ticksinfacet*Scanhead.VARIABLES['FACETS']*Scanhead.VARIABLES['RPM']/60)
        Scanhead.VARIABLES['LASER_HZ'] = Scanhead.VARIABLES['CRYSTAL_HZ']/self.laserticks
        Scanhead.VARIABLES['SPINUP_TIME'] = 10/Scanhead.VARIABLES['CRYSTAL_HZ']
        Scanhead.VARIABLES['STABLE_TIME'] = 30/Scanhead.VARIABLES['CRYSTAL_HZ']
        Scanhead.VARIABLES['START%'] = 2/self.ticksinfacet
        scanbits = 2
        Scanhead.VARIABLES['END%'] = (self.laserticks*scanbits)/self.ticksinfacet + Scanhead.VARIABLES['START%']
        self.sh = Scanhead(board.Platform(), test=True)
        self.sh.submodules.master = SPIMaster(self.sh.spi, data_width=8, 
                        sys_clk_freq=100e6, spi_clk_freq=5e6, with_csr=False)
        self.mach = Machine(connected=False)
        self.mach.sh = self.sh
        self.mach.spi = FakeSpi(self.sh.master)

    def checkreply(self, command, data_expected):
        ''' 
        helper function 
        command is sent and result is checked
        against data_expected
        '''
        data_received = (yield from self.mach.spi.xfer([command]))[0]
        self.assertEqual(data_received, data_expected)

    def state(self, errors=[], state=Scanhead.STATES.STOP):
        ''' 
        helper function 
        gets a state encoding
        error: list as there can be multiple
        '''
        errorstate = 0
        for error in errors: errorstate += pow(2, error)
        val = errorstate+(state<<5)
        return val

    def checkpin(self, pin, value=0):
        '''helper function
        check state of pin'''
        timeout = 0
        while (yield pin) == value:
            timeout += 1
            if timeout>100:
                raise Exception(f"Pin doesnt change state")
            yield

    def test_pwmgeneration(self):
        ''' verify generation of polygon pulse
        '''
        def cpu_side():
            yield from self.checkpin(self.sh.poly_pwm, value=0)
            yield from self.checkpin(self.sh.poly_pwm, value=1)
        #run_simulation(self.sh, [cpu_side()], clocks=self.clocks)

    def test_testmodes(self):
        ''' verify four test modes; laser, motor, line and photodiode
        '''
        def cpu_side():
            test_commands = [Scanhead.COMMANDS.LASERTEST,
                             Scanhead.COMMANDS.MOTORTEST,
                             Scanhead.COMMANDS.LINETEST,
                             Scanhead.COMMANDS.PHOTODIODETEST]
            states = [Scanhead.STATES.LASERTEST,
                      Scanhead.STATES.MOTORTEST,
                      Scanhead.STATES.LINETEST,
                      Scanhead.STATES.PHOTODIODETEST]
            # NOTE: on default photodiode is high
            yield self.sh.photodiode.eq(1) 
            for idx, test_command in enumerate(test_commands):
                yield from self.checkreply(Scanhead.COMMANDS.STATUS, self.state(state=Scanhead.STATES.STOP))
                yield from self.checkreply(test_command, self.state(state=Scanhead.STATES.STOP))
                if test_command != Scanhead.COMMANDS.MOTORTEST:
                    yield from self.checkpin(self.sh.laser0)
                if test_command != Scanhead.COMMANDS.LASERTEST:
                    yield from self.checkpin(self.sh.poly_en, value=1)
                if test_command == Scanhead.COMMANDS.PHOTODIODETEST:
                    yield self.sh.photodiode.eq(0) 
                    # check if laser and polygon turned off
                    yield from self.checkpin(self.sh.laser0, 1)
                    yield from self.checkpin(self.sh.poly_en, value=0)
                    yield from self.checkreply(Scanhead.COMMANDS.STOP, self.state(state=Scanhead.STATES.STOP))
                else:
                    yield from self.checkreply(Scanhead.COMMANDS.STOP, self.state(state=states[idx]))

        #run_simulation(self.sh, [cpu_side()], clocks=self.clocks)

    def getstate(self, fsm):
        '''
        helper functions
        retrieves encoding of state
        '''
        return fsm.decoding[(yield fsm.state)]

    def checkenterstate(self, fsm, state):
        '''
        helper function; checks if state is entered
        '''
        timeout  = 0
        # NOTE: state state is dangerous notation not the same
        while (fsm.decoding[(yield fsm.state)] != state):
            timeout += 1
            if timeout>100:
                raise Exception(f"State not reached")
            yield

    def test_writedata(self):
        def cpu_side():
            for i in range(self.sh.MEMDEPTH+1):
                data_byte = i%256 # bytes can't be larger than 255
                if i%(self.sh.CHUNKSIZE)==0:
                    if (i>0)&((i%self.sh.MEMDEPTH)==0):
                        # check if memory is full
                        yield from self.checkreply(self.sh.COMMANDS.WRITE_L, 
                                                    self.state(errors = [self.sh.ERRORS.MEMFULL],
                                                               state=self.sh.STATES.STOP))
                        continue
                    else:
                        yield from self.checkreply(self.sh.COMMANDS.WRITE_L,
                                                    self.state(state=self.sh.STATES.STOP))
                yield from self.checkreply(data_byte, self.state(state=self.sh.STATES.STOP))
            in_memory = []
            loops = self.sh.MEMDEPTH
            for i in range(loops):
                value = (yield self.sh.mem[i])
                in_memory.append(value)
            self.assertEqual(list(range(loops)),in_memory)
        #run_simulation(self.sh, [cpu_side()])

    def photodiode_trigger(self):
        '''helper functions
           triggers the photodiode for end
           of facet
        '''
        count = (yield self.sh.tickcounter)
        #TODO: remove jitter ticks, should always substract 3!
        for _ in range(self.ticksinfacet-self.sh.JITTERTICKS-1-count): yield
        yield self.sh.photodiode.eq(0)
        yield
        yield
        yield self.sh.photodiode.eq(1)
        yield  

    def test_nosync(self):
        def cpu_side():
            yield self.sh.photodiode.eq(1) 
            yield from self.checkreply(Scanhead.COMMANDS.STATUS, self.state(state=Scanhead.STATES.STOP))
            yield from self.checkreply(Scanhead.COMMANDS.START, self.state(state=Scanhead.STATES.STOP))
            yield from self.checkenterstate(self.sh.laserfsm, 'SPINUP')
            yield from self.checkenterstate(self.sh.laserfsm, 'STATE_WAIT_STABLE')
            yield from self.checkpin(self.sh.laser0, value = 1)
            yield from self.checkenterstate(self.sh.laserfsm, 'STOP')
            yield from self.checkreply(Scanhead.COMMANDS.STATUS,
                                       self.state(errors=[Scanhead.ERRORS.NOTSTABLE],
                                                  state=Scanhead.STATES.STOP))
        #run_simulation(self.sh, [cpu_side()])

    def test_scanlinewithoutwrite(self):
        def cpu_side():
            yield from self.checkreply(Scanhead.COMMANDS.START, self.state(state=Scanhead.STATES.STOP))
            yield from self.checkenterstate(self.sh.laserfsm, 'STATE_WAIT_STABLE')
            yield from self.photodiode_trigger()
            yield from self.checkenterstate(self.sh.laserfsm, 'READ_INSTRUCTION')
            yield from self.checkenterstate(self.sh.laserfsm, 'WAIT_END')
            yield from self.checkenterstate(self.sh.laserfsm, 'STATE_WAIT_STABLE')
            yield from self.photodiode_trigger()
            yield from self.checkreply(Scanhead.COMMANDS.STOP, self.state(errors=[Scanhead.ERRORS.MEMREAD],
                                                                           state=Scanhead.STATES.START))
        #run_simulation(self.sh, [cpu_side()])

    def test_invalidspicommand(self):
        def cpu_side():
            yield from self.checkreply(255, self.state(state=Scanhead.STATES.STOP))
            yield from self.checkreply(Scanhead.COMMANDS.STATUS,
                                        self.state(errors=[Scanhead.ERRORS.INVALID],
                                        state=Scanhead.STATES.STOP))
        #run_simulation(self.sh, [cpu_side()])

    def test_invalidscanline(self):
        ''' verifies error is raised if scanline is sent with invalid command byte
        '''
        def cpu_side():
            yield from self.checkreply(self.sh.COMMANDS.WRITE_L, self.state(state=self.sh.STATES.STOP))
            for _ in range(Scanhead.CHUNKSIZE): yield from self.checkreply(int('11111101', 2),
                                                                self.state(state=Scanhead.STATES.STOP))
            yield self.sh.photodiode.eq(1) 
            yield from self.checkreply(Scanhead.COMMANDS.START, self.state(state=Scanhead.STATES.STOP))
            yield from self.checkenterstate(self.sh.laserfsm, 'STATE_WAIT_STABLE')
            yield from self.photodiode_trigger()
            yield from self.checkenterstate(self.sh.laserfsm, 'WAIT_END')
            yield from self.checkreply(self.sh, self.state(errors=[self.sh.ERRORS.INVALIDLINE],
                                                                   state=Scanhead.STATES.START))
            yield from self.checkenterstate(self.sh.laserfsm, 'STOP')
        #run_simulation(self.scanhead, [cpu_side()])

    def test_stopscanline(self):
        ''' verifies error is raised if scanline is sent with invalid command byte
        '''
        def cpu_side():
            yield from self.mach.writeline([0]*self.sh.BITSINSCANLINE, lastline=True)
            yield from self.checkreply(Scanhead.COMMANDS.START, self.state(state=Scanhead.STATES.STOP))
            yield from self.checkenterstate(self.sh.laserfsm, 'STATE_WAIT_STABLE')
            yield from self.photodiode_trigger()
            yield from self.checkenterstate(self.sh.laserfsm, 'WAIT_END')
            yield from self.checkreply(Scanhead.COMMANDS.STOP, self.state(errors=[Scanhead.ERRORS.INVALIDLINE],
                                                                          state=Scanhead.STATES.START))
            yield from self.checkenterstate(self.sh.laserfsm, 'STOP')
        run_simulation(self.sh, [cpu_side()])

    def test_scanlinewithwrite(self):
        def cpu_side():
            for _ in range(int(Scanhead.MEMDEPTH/Scanhead.MEMWIDTH)): 
                yield from self.checkreply(Scanhead.COMMANDS.WRITE_L, self.state(state=Scanhead.STATES.STOP))
                for _ in range(Scanhead.CHUNKSIZE): yield from self.checkreply(int('11111101', 2), self.state(state=Scanhead.STATES.STOP))
            # check if mem is full
            yield from self.checkreply(Scanhead.COMMANDS.STATUS, self.state(errors=[Scanhead.ERRORS.MEMFULL],
                                                                             state=Scanhead.STATES.STOP))
            # turn on laser head
            yield self.sh.photodiode.eq(1) 
            yield from self.checkreply(Scanhead.COMMANDS.START, self.state(errors=[Scanhead.ERRORS.MEMFULL],
                                                                             state=Scanhead.STATES.STOP))
            # check if statemachine goes to spinup state
            yield from self.checkenterstate(self.sh.laserfsm, 'SPINUP')
            # check if statemachine goes to statewaitstable
            yield from self.checkenterstate(self.sh.laserfsm, 'STATE_WAIT_STABLE')
            # finish cycle
            for _ in range(1): yield from self.photodiode_trigger()
            # counter should be clean before as it enters "WAIT_FOR_DATA_RUN"
            self.assertEqual((yield self.sh.tickcounter), 0)
            self.assertEqual((yield from self.getstate(self.sh.laserfsm)), 'WAIT_FOR_DATA_RUN')
            startwaitticks = int(Scanhead.VARIABLES['START%']*self.ticksinfacet)
            for _ in range(startwaitticks-1): yield
            self.assertEqual((yield self.sh.tickcounter), startwaitticks-1)
            self.assertEqual((yield from self.getstate(self.sh.laserfsm)), 'DATA_RUN')
            self.assertEqual((yield self.sh.laser0), 0)
            self.assertEqual((yield self.sh.scanbit),0)
            self.assertEqual((yield self.sh.lasercnt), 0)
            # You need one tick to start reading first pixel
            yield
            # Pixel 0
            self.assertEqual((yield self.sh.tickcounter), startwaitticks)
            self.assertEqual((yield self.sh.laser0), 1)
            self.assertEqual((yield self.sh.lasercnt), 1)
            yield
            self.assertEqual((yield self.sh.laser0), 1)
            self.assertEqual((yield self.sh.scanbit),1)
            self.assertEqual((yield self.sh.lasercnt), 0)
            yield
            # Pixel 1
            self.assertEqual((yield self.sh.scanbit),2)
            self.assertEqual((yield self.sh.lasercnt), 1)
            self.assertEqual((yield from self.getstate(self.sh.laserfsm)), 'DATA_RUN')
            self.assertEqual((yield self.sh.tickcounter), startwaitticks + self.laserticks)
            self.assertEqual((yield self.sh.laser0), 0)
            yield
            self.assertEqual((yield self.sh.lasercnt), 0)
            self.assertEqual((yield self.sh.laser0), 0)
            yield
            self.assertEqual((yield self.sh.lasercnt), 1)
            self.assertEqual((yield from self.getstate(self.sh.laserfsm)), 'WAIT_END')
            self.assertEqual((yield self.sh.tickcounter), int(Scanhead.VARIABLES['END%']*self.ticksinfacet))
            self.assertEqual((yield self.sh.laser0), 0)
            # check wether it returns to wait state stable
            yield from self.checkenterstate(self.sh.laserfsm, 'STATE_WAIT_STABLE')
            self.assertEqual((yield self.sh.tickcounter), int(self.ticksinfacet*(1-Scanhead.VARIABLES['SYNCSTART'])))
            yield
            self.assertEqual((yield self.sh.laser0), 1)
        #run_simulation(self.scanhead, [cpu_side()])


if __name__ == '__main__':
    unittest.main()
