"""
    spi_memmapping.py
    The memory is initiated with the value 10 in register 0 and the value 20 in register 1.
    As before, Raspberry pi sends two words over SPI. The first word is the command word. 
    The second word is the data write word.
    The command table is as follows;
    command 1 --> self.write value in register 0 and obtain 0
    command 2 --> self.write value in register 1 and obtain 0
    command 3 --> obtain the result of the sum of register 0 and 1.
    else      --> reply with 0
    
    Rik Starmans
"""
import unittest
from migen import *

from litex.soc.cores.spi import SPIMaster, SPISlave

class TestSPI(unittest.TestCase):
    def test_spi_slave_xfer(self):
        class DUT(Module):
            def __init__(self):
                self.specials.mem = Memory(8, 2, init = [10, 20])
                pads = Record([("clk", 1), ("cs_n", 1), ("mosi", 1), ("miso", 1)])
                self.submodules.master = SPIMaster(pads, data_width=8,
                    sys_clk_freq=100e6, spi_clk_freq=5e6,
                    with_csr=False)
                spislave = SPISlave(pads, data_width=8)
                self.submodules.slave = spislave
                self.command = Signal(8)
                # note can flow over
                # je hebt ook de start nodig!!
                # is er een start geweest? ja
                # is er een start en een done geweest ?
                      # zet dan is er een start geweest op nul
                # is er een done geweest
                # zo ja zet de start dan 0

                self.sump = Signal(8)
                self.write = Signal(8)
                done_d = Signal()
                done_rise = Signal()
                self.sync += done_d.eq(spislave.done)
                self.comb += done_rise.eq(spislave.done & ~done_d)
                start_d = Signal()
                start_rise = Signal()
                self.sync += start_d.eq(spislave.start)
                self.comb += start_rise.eq(spislave.start & ~start_d)
                self.comb += self.sump.eq(self.mem[0]+self.mem[1])
                
                self.comb += If(self.command == 1,
                                    self.mem[0].eq(self.write)
                            ).Elif(self.command == 2,
                                    self.mem[1].eq(self.write)
                            )
                self.start_counter = Signal()
                self.sync += \
                    If(start_rise,
                        self.start_counter.eq(1)
                    )
                self.done_counter = Signal()
                self.sync += \
                    If((self.start_counter == 1) & (done_rise == 1),
                        self.done_counter.eq(self.done_counter+1),
                        self.start_counter.eq(0),
                        If(self.done_counter == 0,
                            self.command.eq(spislave.mosi)
                        ).Else(self.write.eq(spislave.mosi))
                    )
                self.comb += If((self.command == 3) & (self.done_counter == 1) ,
                                    spislave.miso.eq(self.sump)
                            ).Else(spislave.miso.eq(0)
                            )

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
            yield from transaction(3, 0)
            yield from transaction(3, 30)
            yield from transaction(1, 0)
            yield from transaction(1, 0)
            yield from transaction(3, 0)
            yield from transaction(3, 21)

            
        def slave_generator(dut):
            def transaction(data_received): 
                while (yield dut.slave.start) == 0:
                    yield
                while (yield dut.slave.done) == 0:
                    yield
                self.assertEqual((yield dut.slave.mosi), data_received)
                self.assertEqual((yield dut.slave.length), 8)
            self.assertEqual((yield dut.done_counter), 0)
            yield from transaction(3)
            yield
            self.assertEqual((yield dut.done_counter), 1)
            #self.assertEqual((yield dut.command), 3)
            #yield from transaction(3, 1)

        dut = DUT()
        run_simulation(dut, [master_generator(dut), slave_generator(dut)], vcd_name="file.vcd")


if __name__ == '__main__':
    unittest.main()






