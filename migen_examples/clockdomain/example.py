import unittest

from migen import *

from hexastorm import board

class CDM(Module):
    def __init__(self, platform):
        clk100 = platform.request('clk100')
        self.clock_domains.cd_sys = ClockDomain(reset_less=True)
        platform.add_period_constraint(self.cd_sys.clk, 10)
        self.clock_domains.cd_slow = ClockDomain(reset_less=True)
        platform.add_period_constraint(self.cd_slow.clk, 50)
        n = 4
        self.counter = Signal(max=n+1)
        self.test = platform.request('led2')
        self.sync += [self.cd_sys.clk.eq(clk100)]
        self.sync += If(self.counter == 0,
                self.cd_slow.clk.eq(~self.cd_slow.clk),
                self.counter.eq(n)).Else(
                self.counter.eq(self.counter - 1))
        self.sync.slow += self.test.eq(~self.test)
        self.testfsm = platform.request('led3')
        self.submodules.fsm = ClockDomainsRenamer('slow')(FSM(reset_state = "RESET"))
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
                self.submodules.cdm = CDM(plat)
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
        run_simulation(self.dut, [steps()], clocks={"sys": 10, 'slow':50})


if __name__ == '__main__':
    import sys
    if len(sys.argv)>1:
        if sys.argv[1] == 'build':
            plat = board.Platform()
            cdm = CDM(plat)
            plat.build(cdm, build_name = 'cdm')
    else:
        unittest.main()