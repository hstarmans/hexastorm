"""
    spi_counter.py
    Creates spi slave where spi slave counts number 
    of messages sent and returns this to master
    
    Rik Starmans
"""
import unittest

from migen import *
from litex.soc.cores.spi import SPIMaster, SPISlave
import sys
sys.path.append("..") 
import icezero as board


class SpiCounter(Module):
    def __init__(self, spi_port, data_width):
        spislave = SPISlave(spi_port, data_width)
        self.submodules.slave = spislave
        done_d = Signal()
        done_rise = Signal()
        self.sync += done_d.eq(spislave.done)
        self.comb += done_rise.eq(spislave.done & ~done_d)
        self.done_counter = Signal(data_width)
        self.sync += \
            If(done_rise,
                self.done_counter.eq(self.done_counter+1),
                spislave.miso.eq(self.done_counter)
            )


class TestSPI(unittest.TestCase):
    def test_spi_slave_xfer(self):
        class DUT(Module):
            def __init__(self):
                pads = Record([("clk", 1), ("cs_n", 1), ("mosi", 1), ("miso", 1)])
                self.submodules.master = SPIMaster(pads, data_width=8,
                    sys_clk_freq=100e6, spi_clk_freq=5e6,
                    with_csr=False)
                self.submodules.spicounter = SpiCounter(pads, data_width=8)

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
                self.assertEqual((yield dut.spicounter.done_counter),i)
                while (yield dut.spicounter.slave.start) == 0:
                    yield
                while (yield dut.spicounter.slave.done) == 0:
                    yield
                self.assertEqual((yield dut.spicounter.slave.mosi), 0xaf)
                self.assertEqual((yield dut.spicounter.slave.length), 8)
                self.assertEqual((yield dut.spicounter.done_counter),i+1)

        dut = DUT()
        run_simulation(dut, [master_generator(dut), slave_generator(dut)])


if __name__ == '__main__':
    import sys
    if len(sys.argv)>1:
        if sys.argv[1] == 'build':
            plat = board.Platform()
            spi_port = plat.request("spi")
            spi_counter = SpiCounter(spi_port, 8)
            plat.build(spi_counter)
    else:
        unittest.main()
    





