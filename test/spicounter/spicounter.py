import sys
import unittest

from nmigen import Signal, Module, Record, Elaboratable
from nmigen.back.pysim import Simulator
from litex.soc.cores.spi import SPIMaster, SPISlave

#from fpgag import board


class SpiCounter(Elaboratable):
    def __init__(pads=None):
        if pads:
            self.test = True
        else:
            self.pads = pads

    def elaborate(self, platform):
        m = Module()
        if not self.test:
            pins = platform.request("spi")
        else:
            pins = self.pads
        m.submodules.slave = spislave = SPISlave(pins,
                                                 data_width=8)
        # Done detector
        done_d = Signal()
        done_rise = Signal()
        m.d.sync += done_d.eq(spislave.done)
        m.d.comb += done_rise.eq(spislave.done & ~done_d)
        done_cntr = Signal(8)
        with m.If(done_rise):
            m.d.comb += [done_cntr.eq(done_cntr+1),
                         spislave.miso.eq(done_cntr)]


class TestSPI(unittest.TestCase):
    def test_spi_slave_xfer(self):
        m = Module()
        pads = Record([("clk", 1), ("cs_n", 1), ("mosi", 1), ("miso", 1)])
        m.submodules.master = SPIMaster(pads, data_width=8,
                                        sys_clk_freq=100e6, spi_clk_freq=5e6,
                                        with_csr=False)
        m.submodules.spicounter = SpiCounter(pads)

        def master_sync():
            for i in range(2):
                yield m.master.mosi.eq(0xaf)
                yield m.master.length.eq(8)
                yield m.master.start.eq(1)
                yield
                yield m.master.start.eq(0)
                yield
                while (yield m.master.done) == 0:
                    yield
                self.assertEqual((yield m.master.miso), i+1)

        def slave_sync():
            for i in range(2):
                self.assertEqual((yield m.spicounter.done_counter), i)
                while (yield m.spicounter.slave.start) == 0:
                    yield
                while (yield m.spicounter.slave.done) == 0:
                    yield
                result = 0xaf if i > 0 else 0
                self.assertEqual((yield m.spicounter.slave.mosi), result)
                self.assertEqual((yield m.spicounter.done_counter), i+1)

        sim = Simulator(m)
        sim.add_process(master_sync)
        sim.add_process(slave_sync)
        sim.run()


if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1:
        if sys.argv[1] == 'build':
            plat = board.Platform()
            spi_port = plat.request("spi")
            spi_counter = SpiCounter(spi_port, 8)
            build_name = 'spi_counter'
            plat.build(spi_counter, build_name=build_name)
            # upload
            plat.upload(build_name)
            # remove build dir
            plat.removebuild()
    else:
        unittest.main()
