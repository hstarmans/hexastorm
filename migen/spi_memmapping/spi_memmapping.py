"""
    spi_memmapping.py
    The memory is initiated with the value 10 in register 0 and the value 20 in register 1.
    The linux host sends one word made up of two bytes over SPI. The first byte is the command. 
    The second byte is optionally the data or ignored.
    The command table is as follows;
    command 1 --> write data in register 0 and reply with (0,1)
    command 2 --> read value from register 1 and reply with (0,value)
    command 3 --> change address and reply with (0,3)
    else --> reply with (0,0)

    Rik Starmans
"""
import unittest
from migen.fhdl import verilog
from migen.fhdl.tools import list_special_ios
from migen import *
from litex.soc.cores.spi import SPIMaster, SPISlave

import sys
sys.path.append("..") 
import icezero as board

class SpiMemmapping(Module):
    def __init__(self, spi_port, data_width):
        # Memory element
        self.specials.mem = Memory(8, 512, init = [10,20])
        p1 = self.mem.get_port(write_capable=True, mode = READ_FIRST)
        p2 = self.mem.get_port(has_re=True)
        self.specials += p1, p2
        self.ios = {p1.adr, p1.dat_w, p1.we, p2.dat_r, p2.adr, p2.re}
        # Receiver state machine
        spislave = SPISlave(spi_port, data_width)
        self.submodules.slave = spislave
        done_d = Signal()
        done_rise = Signal()
        self.sync += done_d.eq(spislave.done)
        self.comb += done_rise.eq(spislave.done & ~done_d)
        start_d = Signal()
        start_rise = Signal()
        self.sync += start_d.eq(spislave.start)
        self.comb += start_rise.eq(spislave.start & ~start_d)
        self.submodules.receiver = FSM(reset_state = "IDLE")
        self.bytecounter = Signal()
        self.receiver.act("IDLE",
            NextValue(p1.we, 0),
            NextValue(p2.re, 1),
            If(start_rise,
                NextState("WAITFORDONE")))
        #TODO: add a timeout and fail
        self.receiver.act("WAITFORDONE",
            If(done_rise,
                NextState("PROCESSINPUT")))
        command = Signal(data_width)
        self.receiver.act("PROCESSINPUT",
            NextValue(self.bytecounter, self.bytecounter+1),
            If(self.bytecounter == 0,
                NextValue(command, spislave.mosi),
                If(spislave.mosi == 2,
                    NextValue(p2.re,0),  
                    NextState("READ")
                ).
                Elif(spislave.mosi == 1,    
                    NextValue(spislave.miso,1),
                    NextState("IDLE")
                ).
                Elif(spislave.mosi == 3,    
                    NextValue(spislave.miso,3),
                    NextState("IDLE")
                ).
                Else(NextValue(spislave.miso,0),
                    NextState("IDLE")
                )
            ).
            Elif((self.bytecounter == 1) & (command == 1),
                NextValue(p1.dat_w, spislave.mosi),
                NextValue(p1.we, 1),
                NextState("WRITE")
            ).
            Elif((self.bytecounter == 1) & (command == 3),
                NextValue(p2.adr, spislave.mosi),
                NextValue(p1.adr, spislave.mosi),
                NextValue(spislave.miso,0),
                NextState("IDLE")
            ).
            Else(
                NextValue(spislave.miso, 0),
                NextState("IDLE")
            )
        )
        self.receiver.act("READ",
            NextValue(spislave.miso, p2.dat_r),
            NextState("IDLE")
        )
        self.receiver.act("WRITE",
            NextValue(p1.we, 0),
            NextValue(spislave.miso, 0),
            NextState("IDLE")
        )


class TestSPI(unittest.TestCase):
    def test_spi_slave_xfer(self):
        class DUT(Module):
            def __init__(self):
                pads = Record([("clk", 1), ("cs_n", 1), ("mosi", 1), ("miso", 1)])
                self.submodules.master = SPIMaster(pads, data_width=8,
                    sys_clk_freq=100e6, spi_clk_freq=5e6,
                    with_csr=False)
                self.submodules.spimemmap = SpiMemmapping(pads, 8)

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
            # read from address 0, check equal to 10
            yield from transaction(2, 0)
            yield from transaction(2, 10)
            # change address to 1
            yield from transaction(3, 0)
            yield from transaction(1, 3)
            # read from address 1, check equal to 20
            yield from transaction(2, 0)
            yield from transaction(2, 20)
            # write 5 to address 1
            yield from transaction(1, 0)
            yield from transaction(5, 1)
            # change address to 0
            yield from transaction(3, 0)
            yield from transaction(0, 3)
            # read from address 0, check equal to 10
            yield from transaction(2, 0)
            yield from transaction(2, 10)
            # change address to 1
            yield from transaction(3, 0)
            yield from transaction(1, 3)
            # read from address 1, check equal to 5 
            yield from transaction(2, 0)
            yield from transaction(2, 5)

        def slave_generator(dut):
            def transaction(data_received): 
                while (yield dut.spimemmap.slave.start) == 0:
                    yield
                while (yield dut.spimemmap.slave.done) == 0:
                    yield
                self.assertEqual((yield dut.spimemmap.slave.mosi), data_received)
                self.assertEqual((yield dut.spimemmap.slave.length), 8)
            self.assertEqual((yield dut.spimemmap.bytecounter), 0)
            yield from transaction(2)
        dut = DUT()
        run_simulation(dut, [master_generator(dut), slave_generator(dut)])


if __name__ == '__main__':
    import sys
    if len(sys.argv)>1:
        if sys.argv[1] == 'build':
            plat = board.Platform()
            spi_port = plat.request("spi")
            spi_memmapping = SpiMemmapping(spi_port, 8)
            plat.build(spi_memmapping, build_name = 'spi_memmapping')
    else:
        unittest.main()