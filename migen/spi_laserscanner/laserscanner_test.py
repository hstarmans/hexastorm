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
                ticksinfacet = 10
                Scanhead.VARIABLES['RPM'] = Scanhead.VARIABLES['CRYSTAL_HZ']/(ticksinfacet*60*Scanhead.VARIABLES['FACETS'])
                Scanhead.MEMDEPTH = 16
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

    def state(self, memory_full=False
                  , error=0, state=Scanhead.STATES.STOP):
        ''' 
        helper function to get a state encoding
        '''
        return int(memory_full)+(error<<1)+(state<<5)

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


if __name__ == '__main__':
    unittest.main()