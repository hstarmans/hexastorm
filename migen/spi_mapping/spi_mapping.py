"""
    spi_mapping.py
    The Raspberry pi sends two words over SPI. The first word is the command word. 
    The second word is the data write word.
    The command table is as follows;
        command 1 --> reply with 2
        command 2 --> reply with 8
        else      --> reply with 0
    The data write word is not used.
    
    Rik Starmans
"""
import unittest

from migen import *
from litex.soc.cores.spi import SPIMaster, SPISlave
import sys
sys.path.append("..") 
import hexa as board


class SpiMapping(Module):
    def __init__(self, spi_port, data_width):
        spislave = SPISlave(spi_port, data_width=8)
        self.submodules.slave = spislave
        command = Signal(8)
        done_d = Signal()
        done_rise = Signal()
        self.sync += done_d.eq(spislave.done)
        self.comb += done_rise.eq(spislave.done & ~done_d)
        start_d = Signal()
        start_rise = Signal()
        self.sync += start_d.eq(spislave.start)
        self.comb += start_rise.eq(spislave.start & ~start_d)
        self.comb += If(command == 1,
                            spislave.miso.eq(2)
                    ).Elif(command == 2,
                            spislave.miso.eq(8)
                    ).Else(spislave.miso.eq(0))
        self.start_counter = Signal()
        self.sync += \
            If(start_rise,
                self.start_counter.eq(1)
            )
        self.done_counter = Signal()
        self.sync += \
                If((self.start_counter == 1 ) & (done_rise == 1),
                    self.done_counter.eq(self.done_counter+1),
                    self.start_counter.eq(0),
                    If(self.done_counter == 0,
                    command.eq(spislave.mosi)
                    )
                )


class TestSPI(unittest.TestCase):
    def test_spi_slave_xfer(self):
        class DUT(Module):
            def __init__(self):
                pads = Record([("clk", 1), ("cs_n", 1), ("mosi", 1), ("miso", 1)])
                self.submodules.master = SPIMaster(pads, data_width=8,
                    sys_clk_freq=100e6, spi_clk_freq=5e6,
                    with_csr=False)
                self.submodules.spimap = SpiMapping(pads, 8)


        def master_generator(dut):
            def transaction(data_sent, data_received):
                yield dut.master.mosi.eq(data_sent)
                yield dut.master.length.eq(8)
                yield dut.master.start.eq(1)
                yield
                yield dut.master.start.eq(0)
                yield
                while (yield dut.master.done) == 0:
                    yield
                self.assertEqual((yield dut.master.miso), data_received)
            yield from transaction(1, 0)
            yield from transaction(1, 2)
            yield from transaction(2, 2)
            yield from transaction(2, 8)


        def slave_generator(dut):
            def transaction(data_received, done_counter):
                self.assertEqual((yield dut.spimap.done_counter),done_counter)
                while (yield dut.spimap.slave.start) == 0:
                    yield
                while (yield dut.spimap.slave.done) == 0:
                    yield
                self.assertEqual((yield dut.spimap.slave.mosi), data_received)
                self.assertEqual((yield dut.spimap.slave.length), 8)
                next_count = 0
                if done_counter == 0:
                    next_count = 1
                yield
                self.assertEqual((yield dut.spimap.done_counter), next_count)
            yield from transaction(1, 0)
            yield from transaction(1, 1)
            yield from transaction(2, 0)
            yield from transaction(2, 1)
        dut = DUT()
        run_simulation(dut, [master_generator(dut), slave_generator(dut)])


if __name__ == '__main__':
    import sys
    if len(sys.argv)>1:
        if sys.argv[1] == 'build':
            plat = board.Platform()
            spi_port = plat.request("spi")
            spi_mapping = SpiMapping(spi_port, 8)
            plat.build(spi_mapping,build_name = 'spi_mapping')         
    else:
        unittest.main()






