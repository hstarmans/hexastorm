import unittest

from migen import *
from litex.soc.cores.clock import iCE40PLL

from hexastorm import board

class CDM(Module):
    def __init__(self, platform, test=False):
        clk100 = platform.request('clk100')
        self.clock_domains.cd_sys = ClockDomain(reset_less=True)
        self.sync += [self.cd_sys.clk.eq(clk100)]
        platform.add_period_constraint(self.cd_sys.clk, 10)
        #rst_n = platform.request("rst_n")
        self.clock_domains.cd_slow = ClockDomain(reset_less=True)
        platform.add_period_constraint(self.cd_slow.clk, 50)
        n = 4
        self.counter = Signal(max=n+1)
        self.sync += If(self.counter == 0,
                self.cd_slow.clk.eq(~self.cd_slow.clk),
                self.counter.eq(n)).Else(
                self.counter.eq(self.counter - 1))
        self.clock_domains.cd_slow_pll = ClockDomain(reset_less=True)
        platform.add_period_constraint(self.cd_slow_pll.clk, 50)
        if not test:
            self.submodules.pll = pll = iCE40PLL()
            #self.comb += pll.reset.eq(~rst_n)
            pll.register_clkin(clk100, 100e6)
            pll.create_clkout(self.cd_slow_pll, 20e6)
        self.test = platform.request('led2')
        self.sync.slow += self.test.eq(~self.test)
        self.testfsm = platform.request('led3')
        self.submodules.fsm = ClockDomainsRenamer('slow_pll')(FSM(reset_state = "RESET"))
        self.fsm.act("RESET",
                NextState("IDLE")
        )
        self.fsm.act("IDLE",
                NextValue(self.testfsm, ~self.testfsm)
        )


class TestCDM(unittest.TestCase):
    def setUp(self):
        class DUT(Module):
            def __init__(self):
                plat = board.Platform()
                self.submodules.cdm = CDM(plat, test=True)
        self.dut = DUT()

    def test_count(self):
        def steps():
            for _ in range(3):
                self.assertEqual((yield self.dut.cdm.testfsm), 0)
                self.assertEqual((yield self.dut.cdm.test), 0)
                yield
            for _ in range(5):
                self.assertEqual((yield self.dut.cdm.testfsm), 0) 
                self.assertEqual((yield self.dut.cdm.test), 1)
                yield
            for _ in range(5):
                self.assertEqual((yield self.dut.cdm.testfsm), 1)  
                self.assertEqual((yield self.dut.cdm.test), 0)
                yield
            for _ in range(5):
                self.assertEqual((yield self.dut.cdm.testfsm), 0)  
                self.assertEqual((yield self.dut.cdm.test), 1)
                yield
        run_simulation(self.dut, [steps()], clocks={"sys": 10, 'slow':50, 'slow_pll':50})


if __name__ == '__main__':
    import sys
    if len(sys.argv)>1:
        if sys.argv[1] == 'build':
            plat = board.Platform()
            cdm = CDM(plat)
            plat.build(cdm, build_name = 'cdm')
    else:
        unittest.main()