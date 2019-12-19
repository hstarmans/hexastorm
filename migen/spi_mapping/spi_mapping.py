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
                command = Signal(8)
                done_d = Signal()
                done_rise = Signal()
                self.sync += done_d.eq(spislave.done)
                self.comb += done_rise.eq(spislave.done & ~done_d)
                self.comb += If(command == 1,
                                    spislave.miso.eq(2)
                            ).Elif(command == 2,
                                    spislave.miso.eq(8)
                            ).Else(spislave.miso.eq(0))
# stel je done counter kan alleen 0 of 1 zijn
# op elke done rise doe je dan nog steeds de done counter verhogen, deze flipt dus tussen
# 0 en 1.
# maar als done counter is nul dan is het signaal wat je ontvangt het commando
# het commando is on default 0 maar dit wordt dus meteen geupdate na een rise
# het probleem bij deze is dat als er iets mis gaat, 
                self.done_counter = Signal()
                self.sync += \
                    If(done_rise,
                        self.done_counter.eq(self.done_counter+1)
                    )
                self.sync += \
                    If(done_rise & self.done_counter ==0,
                        command.eq(spislave.mosi)
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



        # def master_generator(dut):
        #     def transaction(data_sent, data_received):
        #         yield dut.master.mosi.eq(data_sent)
        #         yield dut.master.length.eq(8)
        #         yield dut.master.start.eq(1)
        #         yield
        #         yield dut.master.start.eq(0)
        #         yield
        #         while (yield dut.master.done) == 0:
        #             yield
        #         self.assertEqual((yield dut.master.miso), data_received)
        #     transaction(1, 0)
        #     transaction(1, 2)


        # def slave_generator(dut):
        #     def transaction(data_received, done_counter):
        #         self.assertEqual((yield dut.done_counter),done_counter)
        #         while (yield dut.slave.start) == 0:
        #             yield
        #         while (yield dut.slave.done) == 0:
        #             yield
        #         self.assertEqual((yield dut.slave.mosi), data_received)
        #         self.assertEqual((yield dut.slave.length), 8)
        #         next_count = 0
        #         if done_counter == 0:
        #             next_count = 1
        #         self.assertEqual((yield dut.done_counter), next_count)
        #     transaction(1, 0)
        #     transaction(1, 0)

        dut = DUT()
        run_simulation(dut, [master_generator(dut), slave_generator(dut)])


if __name__ == '__main__':
    unittest.main()






