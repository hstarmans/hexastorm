import unittest

from migen import *
from litex.soc.cores.spi import SPIMaster

from hexastorm.core import Scanhead


class TestScanhead(unittest.TestCase):
    
    def setUp(self):
        class DUT(Module):
            def __init__(self):
                pads = Record([("clk", 1), ("cs_n", 1), ("mosi", 1), ("miso", 1)])
                self.submodules.master = SPIMaster(pads, data_width=8,
                        sys_clk_freq=100e6,
                        spi_clk_freq=5e6,
                        with_csr=False)
                self.laser0, self.poly_en, self.poly_pwm = Signal(), Signal(), Signal()
                self.photodiode = Signal()
                # you want to alter the clock --> and let the rest stay stable
                self.ticksinfacet = 12
                self.laserticks = 2
                Scanhead.VARIABLES['SYNCSTART'] = 0.2
                Scanhead.VARIABLES['CRYSTAL_HZ']= self.ticksinfacet*60*Scanhead.VARIABLES['FACETS']*Scanhead.VARIABLES['RPM']
                Scanhead.VARIABLES['LASER_HZ'] = Scanhead.VARIABLES['CRYSTAL_HZ']/self.laserticks
                Scanhead.VARIABLES['JITTER_THRESH'] = 0.1 # need to have at least one tick play
                Scanhead.VARIABLES['SPINUP_TIME'] = 10/Scanhead.VARIABLES['CRYSTAL_HZ']
                Scanhead.VARIABLES['STABLE_TIME'] = 30/Scanhead.VARIABLES['CRYSTAL_HZ']
                Scanhead.VARIABLES['START%'] = 2/self.ticksinfacet
                scanbits = 2
                Scanhead.VARIABLES['SYNCSTART'] = 2/self.ticksinfacet
                Scanhead.VARIABLES['END%'] = (self.laserticks*scanbits)/self.ticksinfacet + Scanhead.VARIABLES['START%']
                # NOTE: if memdepth is not divisable by eight, your code does not work!
                Scanhead.MEMDEPTH = 32
                self.submodules.scanhead = Scanhead(pads,
                                                    self.laser0,
                                                    self.poly_pwm,
                                                    self.poly_en,
                                                    self.photodiode)
        self.dut = DUT()
    
    def transaction(self, data_sent, data_received):
        ''' 
        helper function to test transaction from raspberry pi side
        '''
        yield self.dut.master.mosi.eq(data_sent)
        yield self.dut.master.length.eq(8)
        yield self.dut.master.start.eq(1)
        yield
        yield self.dut.master.start.eq(0)
        yield
        while (yield self.dut.master.done) == 0:
            yield
        self.assertEqual((yield self.dut.master.miso), data_received)

    def state(self, errors=[], state=Scanhead.STATES.STOP):
        ''' 
        helper function to get a state encoding
        error: list as there can be multiple
        '''
        errorstate = 0
        for error in errors: errorstate += pow(2, error)
        val = errorstate+(state<<5)
        return val

    def checkpin(self, pin, value=0): 
        timeout = 0
        while (yield pin) == value:
            timeout += 1
            if timeout>100:
                raise Exception(f"Pin doesnt change state")
            yield

    def test_pwmgeneration(self):
        ''' polygon pulse should always be generated
        '''
        def cpu_side():
            yield from self.checkpin(self.dut.poly_pwm, value=0)
            yield from self.checkpin(self.dut.poly_pwm, value=1)
        
        run_simulation(self.dut, [cpu_side()])

    def test_testmodes(self):
        ''' test all 4 testmodes
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
                    
            for idx, test_command in enumerate(test_commands):
                # get the initial status
                yield from self.transaction(Scanhead.COMMANDS.STATUS, self.state(state=Scanhead.STATES.STOP))
                # run a test
                yield from self.transaction(test_command, self.state(state=Scanhead.STATES.STOP))
                if test_command != Scanhead.COMMANDS.MOTORTEST:
                    # check if laser turned on
                    yield from self.checkpin(self.dut.laser0)
                if test_command != Scanhead.COMMANDS.LASERTEST:
                    # check if polygon is turned on
                    yield from self.checkpin(self.dut.poly_en, value=1)
                if test_command == Scanhead.COMMANDS.PHOTODIODETEST:
                    # turn on the photodiode
                    yield self.dut.photodiode.eq(1) 
                    # check if laser and polygon turned off
                    yield from self.checkpin(self.dut.laser0, 1)
                    yield from self.checkpin(self.dut.poly_en, value=0)
                # put the scanhead back in OFF mode and check state trigger 
                yield from self.transaction(Scanhead.COMMANDS.STOP, self.state(state=states[idx]))

        run_simulation(self.dut, [cpu_side()])

    def getstate(self, fsm):
        return fsm.decoding[(yield fsm.state)]

    def checkenterstate(self, fsm, state):
        timeout  = 0
        # NOTE: state state is dangerous notation not the same
        while (fsm.decoding[(yield fsm.state)] != state):
            timeout += 1
            if timeout>100:
                raise Exception(f"State not reached")
            yield

    def test_writedata(self):
        def cpu_side():
            for i in range(Scanhead.MEMDEPTH+1):
                data_byte = i%256 # bytes can't be larger than 255
                if i%(Scanhead.CHUNKSIZE)==0:
                    if (i>0)&((i%Scanhead.MEMDEPTH)==0):
                        # check if memory is full
                        yield from self.transaction(Scanhead.COMMANDS.WRITE_L, 
                                                    self.state(errors = [Scanhead.ERRORS.MEMFULL],
                                                               state=Scanhead.STATES.STOP)
                                                    )
                        continue
                    else:
                        yield from self.transaction(Scanhead.COMMANDS.WRITE_L, 
                                                    self.state(state=Scanhead.STATES.STOP)
                                                    )
                yield from self.transaction(data_byte, self.state(state=Scanhead.STATES.STOP))
            # memory is tested in litex
            in_memory = []
            loops = 10
            for i in range(loops):
                value = (yield self.dut.scanhead.mem[i])
                in_memory.append(value)
            self.assertEqual(list(range(loops)),in_memory)
        run_simulation(self.dut, [cpu_side()])


    def test_scanlinewithoutwrite(self):
        def cycle():
            # off
            for _ in range(self.dut.ticksinfacet-3): yield
            # 3 final clocks create photodiode trigger
            yield self.dut.photodiode.eq(1)  
            yield
            yield self.dut.photodiode.eq(0)
            yield  # falling edge
            yield  # counter reset
            
        def cpu_side():
            # get the initial status
            yield from self.transaction(Scanhead.COMMANDS.STATUS, self.state(state=Scanhead.STATES.STOP))
            # turn on laser head
            yield from self.transaction(Scanhead.COMMANDS.START, self.state(state=Scanhead.STATES.STOP))
            # check if statemachine goes to spinup state
            yield from self.checkenterstate(self.dut.scanhead.laserfsm, 'SPINUP')
            # check if statemachine goes to statewaitstable
            yield from self.checkenterstate(self.dut.scanhead.laserfsm, 'STATE_WAIT_STABLE')
            yield from self.checkpin(self.dut.laser0, value = 1)
            # no trigger on photodiode, so starting scanhead fails and machine returns to stop
            yield from self.checkenterstate(self.dut.scanhead.laserfsm, 'STOP')
            # check if error is received, again turn on laser head
            yield from self.transaction(Scanhead.COMMANDS.START, self.state(errors=[Scanhead.ERRORS.MEMREAD,Scanhead.ERRORS.NOTSTABLE],
                                                                            state=Scanhead.STATES.STOP))
            # check if statemachine goes to statewaitstable
            yield from self.checkenterstate(self.dut.scanhead.laserfsm, 'STATE_WAIT_STABLE')
            # create an undefined starting fase but now apply a pulse
            # system should reach WAIT_FOR _DATA_RUN
            for _ in range(2): yield  #TODO: this can't be random number between 0 and 10
            for _ in range(2): yield from cycle()
            yield from self.checkenterstate(self.dut.scanhead.laserfsm, 'WAIT_FOR_DATA_RUN')
            yield from self.checkenterstate(self.dut.scanhead.laserfsm, 'DATA_RUN')
            # data run should raise error cannot read (nothing in memory yet)
            for _ in range(self.dut.laserticks+1): yield
            # turn off laser head
            # checks wether MEMREAD is triggered and ERROR is gone after reading
            yield from self.transaction(Scanhead.COMMANDS.STOP, self.state(errors=[Scanhead.ERRORS.MEMREAD],
                                                                           state=Scanhead.STATES.START))
        run_simulation(self.dut, [cpu_side()])

    def test_scanlinewithwrite(self):
        def cycle():
            # off
            for _ in range(self.dut.ticksinfacet-3): yield
            # 3 final clocks create photodiode trigger
            yield self.dut.photodiode.eq(1)  
            yield
            yield self.dut.photodiode.eq(0)
            yield  # falling edge
            yield  # counter reset

        def cpu_side():
            for _ in range(int(Scanhead.MEMDEPTH/Scanhead.MEMWIDTH)): 
                yield from self.transaction(Scanhead.COMMANDS.WRITE_L, self.state(state=Scanhead.STATES.STOP))
                for _ in range(Scanhead.CHUNKSIZE): yield from self.transaction(int('11111101', 2), self.state(state=Scanhead.STATES.STOP))
            # quick check if you reached end of chunk
            yield from self.transaction(255, self.state(errors=[Scanhead.ERRORS.MEMFULL], state=Scanhead.STATES.STOP))
            yield from self.transaction(Scanhead.COMMANDS.STATUS, self.state(errors=[Scanhead.ERRORS.INVALID, Scanhead.ERRORS.MEMFULL],
                                                                             state=Scanhead.STATES.STOP))
            # turn on laser head
            yield from self.transaction(Scanhead.COMMANDS.START, self.state(errors=[Scanhead.ERRORS.MEMFULL], state=Scanhead.STATES.STOP))
            # check if statemachine goes to spinup state
            yield from self.checkenterstate(self.dut.scanhead.laserfsm, 'SPINUP')
            # check if statemachine goes to statewaitstable
            yield from self.checkenterstate(self.dut.scanhead.laserfsm, 'STATE_WAIT_STABLE')
            # toggle diode so head stabilizes
            for _ in range(1): yield from cycle()
            # counter should be clean before as it enters "WAIT_FOR_DATA_RUN"
            self.assertEqual((yield self.dut.scanhead.tickcounter), 0)
            self.assertEqual((yield from self.getstate(self.dut.scanhead.laserfsm)), 'WAIT_FOR_DATA_RUN')
            startwaitticks = int(Scanhead.VARIABLES['START%']*self.dut.ticksinfacet)
            for _ in range(startwaitticks-1): yield
            self.assertEqual((yield self.dut.scanhead.tickcounter), startwaitticks-1)
            self.assertEqual((yield from self.getstate(self.dut.scanhead.laserfsm)), 'DATA_RUN')
            self.assertEqual((yield self.dut.laser0), 0)
            self.assertEqual((yield self.dut.scanhead.scanbit),0)
            self.assertEqual((yield self.dut.scanhead.lasercnt), 0)
            # You need one tick to start reading first pixel
            yield
            # Pixel 0
            self.assertEqual((yield self.dut.scanhead.tickcounter), startwaitticks)
            self.assertEqual((yield self.dut.laser0), 1)
            self.assertEqual((yield self.dut.scanhead.lasercnt), 1)
            yield
            self.assertEqual((yield self.dut.laser0), 1)
            self.assertEqual((yield self.dut.scanhead.scanbit),1)
            self.assertEqual((yield self.dut.scanhead.lasercnt), 0)
            yield
            # Pixel 1
            self.assertEqual((yield self.dut.scanhead.scanbit),2)
            self.assertEqual((yield self.dut.scanhead.lasercnt), 1)
            self.assertEqual((yield from self.getstate(self.dut.scanhead.laserfsm)), 'DATA_RUN')
            self.assertEqual((yield self.dut.scanhead.tickcounter), startwaitticks + self.dut.laserticks)
            self.assertEqual((yield self.dut.laser0), 0)
            yield
            self.assertEqual((yield self.dut.scanhead.lasercnt), 0)
            self.assertEqual((yield self.dut.laser0), 0)
            yield
            self.assertEqual((yield self.dut.scanhead.lasercnt), 1)
            self.assertEqual((yield from self.getstate(self.dut.scanhead.laserfsm)), 'WAIT_END')
            self.assertEqual((yield self.dut.scanhead.tickcounter), int(Scanhead.VARIABLES['END%']*self.dut.ticksinfacet))
            self.assertEqual((yield self.dut.laser0), 0)
            # check wether it returns to wait state stable
            yield from self.checkenterstate(self.dut.scanhead.laserfsm, 'STATE_WAIT_STABLE')
            self.assertEqual((yield self.dut.scanhead.tickcounter), int(self.dut.ticksinfacet*(1-Scanhead.VARIABLES['SYNCSTART'])))
            yield
            self.assertEqual((yield self.dut.laser0), 1)
            print((yield self.dut.scanhead.tickcounter))

        run_simulation(self.dut, [cpu_side()])


if __name__ == '__main__':
    unittest.main()