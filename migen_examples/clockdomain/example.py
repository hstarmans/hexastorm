import unittest

from migen import *

from hexastorm import board

class CDM(Module):
    def __init__(self):
        self.clock_domains.cd_slow = ClockDomain(reset_less=True)
        maxcount = 4
        self.counter = Signal(max=maxcount)
        self.test = Signal()
        self.sync += If(self.counter == 0,
                self.cd_slow.clk.eq(~self.cd_slow.clk),
                self.counter.eq(maxcount)).Else(
                self.counter.eq(self.counter - 1))
        self.sync.slow += self.test.eq(~self.test)
        self.testfsm = Signal()
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
                self.submodules.cdm = CDM()
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
            cdm = CDM()
            plat.build(cdm, build_name = 'cdm')
    else:
        unittest.main()