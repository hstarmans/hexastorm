import unittest

from migen import *
from litex.soc.cores.spi import SPIMaster
from spi_laserscanner import Scanhead


class TestSpiLaserScanner(unittest.TestCase):
    
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
                self.ticksinfacet = 10
                self.laserticks = 2
                Scanhead.VARIABLES['CRYSTAL_HZ']= self.ticksinfacet*60*Scanhead.VARIABLES['FACETS']*Scanhead.VARIABLES['RPM']
                Scanhead.VARIABLES['LASER_HZ'] = Scanhead.VARIABLES['CRYSTAL_HZ']/self.laserticks
                Scanhead.VARIABLES['JITTER_THRESH'] = 0.1 # need to have at least one tick play
                Scanhead.VARIABLES['SPINUP_TIME'] = 10/Scanhead.VARIABLES['CRYSTAL_HZ']
                Scanhead.VARIABLES['STABLE_TIME'] = 30/Scanhead.VARIABLES['CRYSTAL_HZ']
                Scanhead.VARIABLES['START%'] = 0.3
                Scanhead.MEMDEPTH = 24
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
        
        #run_simulation(self.dut, [cpu_side()])

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
            # write lines to memory
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
        #run_simulation(self.dut, [cpu_side()])


    def test_scanlinewithoutwrite(self):
        ''' test scanhead
        '''
        def cycle():
            for _ in range(self.dut.ticksinfacet-2): yield
            yield self.dut.photodiode.eq(1)
            yield
            yield self.dut.photodiode.eq(0)
            
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
            yield from self.transaction(Scanhead.COMMANDS.START, self.state(errors=[Scanhead.ERRORS.NOTSTABLE],
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
        #run_simulation(self.dut, [cpu_side()])

    def test_scanlinewithwrite(self):
        def cycle():
            #NOTE: yield is a clock cycle and yield signal is not a clock cycle!!
            # 8 clocks off
            for _ in range(self.dut.ticksinfacet-3): yield
            # 2 final clocks create photodiode trigger
            yield self.dut.photodiode.eq(1)  
            yield
            yield self.dut.photodiode.eq(0)
            yield  # falling edge
            yield  # counter reset

        def cpu_side():
            for _ in range(int(Scanhead.MEMDEPTH/Scanhead.MEMWIDTH)): 
                yield from self.transaction(Scanhead.COMMANDS.WRITE_L, self.state(state=Scanhead.STATES.STOP))
                for _ in range(Scanhead.CHUNKSIZE): yield from self.transaction(255, self.state(state=Scanhead.STATES.STOP))
            # quick check if you reached end of chunk
            yield from self.transaction(255, self.state(state=Scanhead.STATES.STOP))
            yield from self.transaction(Scanhead.COMMANDS.STATUS, self.state(errors=[Scanhead.ERRORS.INVALID],
                                                                             state=Scanhead.STATES.STOP))
            # turn on laser head
            yield from self.transaction(Scanhead.COMMANDS.START, self.state(state=Scanhead.STATES.STOP))
            # check if statemachine goes to spinup state
            yield from self.checkenterstate(self.dut.scanhead.laserfsm, 'SPINUP')
            # check if statemachine goes to statewaitstable
            yield from self.checkenterstate(self.dut.scanhead.laserfsm, 'STATE_WAIT_STABLE')
            # toggle diode so head stabilizes
            for _ in range(2): yield from cycle()
            # counter should be clean before as it enters "WAIT_FOR_DATA_RUN"
            self.assertEqual((yield self.dut.scanhead.tickcounter), 0)
            self.assertEqual((yield from self.getstate(self.dut.scanhead.laserfsm)), 'WAIT_FOR_DATA_RUN')
            #yield from self.checkenterstate(self.dut.scanhead.laserfsm, 'WAIT_FOR_DATA_RUN')
            for _ in range(int(Scanhead.VARIABLES['START%']*self.dut.ticksinfacet)): yield
            # wait before line starts
            self.assertEqual((yield self.dut.scanhead.tickcounter), int(Scanhead.VARIABLES['START%']*self.dut.ticksinfacet))
            self.assertEqual((yield from self.getstate(self.dut.scanhead.laserfsm)), 'READ')
            yield
            self.assertEqual((yield from self.getstate(self.dut.scanhead.laserfsm)), 'DATA_RUN')
            yield
            self.assertEqual((yield self.dut.laser0), 1)
            # # at the fourth tick you are in data run S
            # print((yield self.dut.scanhead.tickcounter))
            
            # yield # lezen
            # yield
            # yield
            # yield
            # yield
            # print((yield self.dut.scanhead.error))
            # print((yield from self.getstate(self.dut.scanhead.laserfsm)))
            #self.assertEqual((yield from self.getstate(self.dut.scanhead.laserfsm)), 'READ')


        run_simulation(self.dut, [cpu_side()])


#OKEY WELKE TESTEN WIL JE NOG
# schrijf naar geheugen en kijk of laser op juiste tijd aangaat
# check of laser aangaat voor einde

# onderste twee kun je combineren
#  check single facet mode
#  bouw een single line mode

if __name__ == '__main__':
    unittest.main()