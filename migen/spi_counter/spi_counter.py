"""
    spi_counter.py
    Creates spi slave where spi slave counts number 
    of messages sent and returns this to master
    
    Rik Starmans
"""
import unittest

from migen import *

from litex.soc.cores.spi import SPIMaster, SPISlave

class TestSPI(unittest.TestCase):
    def test_spi_slave_xfer(self):
        class DUT(Module):
            def __init__(self):
                pads = Record([("clk", 1), ("cs_n", 1), ("mosi", 1), ("miso", 1)])
                self.submodules.master = SPIMaster(pads, data_width=8,
                    sys_clk_freq=100e6, spi_clk_freq=5e6,
                    with_csr=False)
                spislave = SPISlave(pads, data_width=8)
                self.submodules.slave = spislave
                done_d = Signal()
                done_rise = Signal()
                self.sync += done_d.eq(spislave.done)
                self.comb += done_rise.eq(spislave.done & ~done_d)
                self.done_counter = Signal(8)
                self.sync += \
                    If(done_rise,
                        self.done_counter.eq(self.done_counter+1),
                        spislave.miso.eq(self.done_counter)
                    )

        def master_generator(dut):
            for i in range(0,2):
                yield dut.master.mosi.eq(0xaf)
                yield dut.master.length.eq(8)
                yield dut.master.start.eq(1)
                yield
                yield dut.master.start.eq(0)
                yield
                while (yield dut.master.done) == 0:
                    yield
                self.assertEqual((yield dut.master.miso), i)

        def slave_generator(dut):
            for i in range(0,2):
                self.assertEqual((yield dut.done_counter),i)
                while (yield dut.slave.start) == 0:
                    yield
                while (yield dut.slave.done) == 0:
                    yield
                self.assertEqual((yield dut.slave.mosi), 0xaf)
                self.assertEqual((yield dut.slave.length), 8)
                self.assertEqual((yield dut.done_counter),i+1)

        dut = DUT()
        run_simulation(dut, [master_generator(dut), slave_generator(dut)])


if __name__ == '__main__':
    unittest.main()






