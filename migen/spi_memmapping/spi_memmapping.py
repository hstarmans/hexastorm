"""
    spi_memmapping.py
    The memory is initiated with the value 10 in register 0 and the value 20 in register 1.
    As before, Raspberry pi sends one word made up of two bytes over SPI. The first byte is the command. 
    The second byte is optionally the data or ignored.
    The command table is as follows;
    command 1 --> write data in register 0 and reply with (0,0)
    command 2 --> read value from register 1 and reply with (0,value)
    command 3 --> change address
    else --> reply with (0,0)

    Rik Starmans
"""
import unittest
from migen.fhdl import verilog
from migen import *
from litex.soc.cores.spi import SPIMaster, SPISlave

import sys
sys.path.append("..") 
import icezero as board


# Er zijn twee state machines, je zou de verwerker en ontvanger achter elkaar kunnen koppelen
# in een state machine of je maakt er twee aparte van.
# Het verschil is niet heel erg groot, de toestanden blijven ze worden alleen anders uitgevoerd
# De verwerker kan met start en done werken, nu laat je hem werken met aan en uit
# Een nadeel is dat twee verschillende objecten dan schijven op een object kunnen lezen en schrijven

# Ontvanger
# toestand 1 is idle, wacht op start
# toestand 2 is start ontvangen, wacht op done of na timeout ga naar idle
# toestand 3 is done ontvangen, verwerk input 
#            als verwerker klaar is
#                update data received en word counter
#                zet miso op antwoord
#                ga terug naar 1

# Verwerker
# Toestand 1: is idle, wacht op start
# Toestand 2: is start ontvangen, maak een beslissing op basis van woordnummer
# Woordnummer 1 & commando 2
#                lees data uit en zet antwoord op uitgelezen data
#                geef aan input verwerkt en ga terug naar 1
# Woordnummer 2 & commando 1 
#                schrijf data en zet antwoord op nul
#                geef aan input verwerkt en ga terug naar 1
# Anders
#                zet antwoord op nul
#                geef aan input verwerkt en ga terug naar 1


# SRAM heeft de volgende eigenschappen
# adres 18 to 0 register
# dout  15 to 0 register
# din   15 to 0 wire
# waarom is din een wire --> er is een verschil in verilog dit verschil is er in 
# in migen niet
# sram ce  --> waarschijnlijk chip select, moet laag zijn om hem te selecteren
# lb en ub worden altijd op nul gezet bij elke klokpuls
# er is verder een sram_oe en sram_we
# laag om te gebruiken, een moet laag zijn anderen hoog
# spi gebruikt resynchronization dit lijkt me niet nodig


class SpiMemmapping(Module):
    def __init__(self, spi_port, sram, data_width):
        # Memory element
        #self.specials.mem = Memory(data_width, 2, init = [10, 20])
        #p1 = self.mem.get_port(write_capable=True)
        #p2 = self.mem.get_port(has_re=True)
        #self.specials += p1, p2
        #self.ios = {p1.adr, p1.dat_w, p1.we, p2.dat_r, p2.adr, p2.re}
        
        sram_dout = Signal(16)
        sram_din = Signal(16)
        sram_adr = Signal(19)
        
        # https://github.com/cliffordwolf/icestorm/blob/master/icefuzz/tests/sb_io.v
        pll= Instance("SB_IO", name = 'sram_dio [15:0]',
                                        p_PIN_TYPE=0b101001,
                                        p_PULLUP=0b0,
                                        # hij maakt hier alles 1 bit wide dit verklaart waarom je een 
                                        # 1 krijgt
                                        io_PACKAGE_PIN=sram.dat,
                                        i_OUTPUT_ENABLE=sram.oe,
                                        i_D_OUT_0 = sram_dout,
                                        o_D_IN_0 = sram_din
                                        ),
        self.specials += [ pll]

        # dit doe je altijd
        self.sync += sram.adr.eq(sram_adr)
        self.sync += sram.lb.eq(0)
        self.sync += sram.ub.eq(0)
        self.sync += sram.cs.eq(0)
        
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
            NextValue(sram.oe, 1),
            NextValue(sram.we, 1),
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
                    NextValue(sram.oe,0),
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
                NextValue(sram_dout, spislave.mosi),
                NextValue(sram.we, 1),
                NextState("WRITE")
            ).
            Elif((self.bytecounter == 1) & (command == 3),
                NextValue(sram_adr, spislave.mosi),
                NextValue(spislave.miso,0),
                NextState("IDLE")
            ).
            Else(
                NextValue(spislave.miso, 0),
                NextState("IDLE")
            )
        )
        self.receiver.act("READ",
            NextValue(sram.oe, 0),
            NextValue(spislave.miso, sram_din),
            NextState("IDLE")
        )
        self.receiver.act("WRITE",
            NextValue(sram.we, 0),
            NextValue(spislave.miso, 0),
            NextState("IDLE")
        )


# class SpiMemmapping(Module):
#     def __init__(self, spi_port, data_width):
#         # Memory element
#         self.specials.mem = Memory(data_width, 2, init = [10, 20])
#         p1 = self.mem.get_port(write_capable=True)
#         p2 = self.mem.get_port(has_re=True)
#         self.specials += p1, p2
#         self.ios = {p1.adr, p1.dat_w, p1.we, p2.dat_r, p2.adr, p2.re}
#         # Receiver state machine
#         spislave = SPISlave(spi_port, data_width)
#         self.submodules.slave = spislave
#         done_d = Signal()
#         done_rise = Signal()
#         self.sync += done_d.eq(spislave.done)
#         self.comb += done_rise.eq(spislave.done & ~done_d)
#         start_d = Signal()
#         start_rise = Signal()
#         self.sync += start_d.eq(spislave.start)
#         self.comb += start_rise.eq(spislave.start & ~start_d)
#         self.submodules.receiver = FSM(reset_state = "IDLE")
#         self.bytecounter = Signal()
#         self.receiver.act("IDLE",
#             If(start_rise,
#                 NextState("WAITFORDONE")))
#         #TODO: add a timeout and fail
#         self.receiver.act("WAITFORDONE",
#             If(done_rise,
#                 NextState("PROCESSINPUT")))
#         command = Signal(data_width)
#         self.receiver.act("PROCESSINPUT",
#             NextValue(self.bytecounter, self.bytecounter+1),
#             If(self.bytecounter == 0,
#                 NextValue(command, spislave.mosi),
#                 If(spislave.mosi == 2,    
#                     NextValue(p2.adr, 0),
#                     NextValue(p2.re,1),
#                     NextState("READ")
#                 ).
#                 Else(NextValue(spislave.miso,0),
#                     NextState("IDLE")
#                 )
#             ).
#             Elif((self.bytecounter == 1) & (command == 1),
#                 NextValue(p1.adr, 0),
#                 NextValue(p1.dat_w, spislave.mosi),
#                 NextValue(p1.we, 1),
#                 NextState("WRITE")
#             ).
#             Else(
#                 NextValue(spislave.miso, 0),
#                 NextState("IDLE")
#             )
#         )
#         self.receiver.act("READ",
#             NextValue(p2.re, 0),
#             NextValue(spislave.miso, p2.dat_r),
#             NextState("IDLE")
#         )
#         self.receiver.act("WRITE",
#             NextValue(p1.we, 0),
#             NextValue(spislave.miso, 0),
#             NextState("IDLE")
#         )


class TestSPI(unittest.TestCase):
    def test_spi_slave_xfer(self):
        class DUT(Module):
            def __init__(self):
                sram = Record([("clk", 1), ("cs_n", 1), ("mosi", 1), ("miso", 1)])
                self.submodules.master = SPIMaster(sram, data_width=8,
                    sys_clk_freq=100e6, spi_clk_freq=5e6,
                    with_csr=False)
                self.submodules.spimemmap = SpiMemmapping(sram, 8)


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
            yield from transaction(2, 0)
            yield from transaction(2, 10)
            yield from transaction(1, 0)
            yield from transaction(15, 0)
            yield from transaction(2, 0)
            yield from transaction(2, 15)
            
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
            while (yield dut.spimemmap.bytecounter) == 0:
                yield
            #self.assertEqual((yield dut.command), 3)
            #yield from transaction(3, 1)

        dut = DUT()
        run_simulation(dut, [master_generator(dut), slave_generator(dut)], vcd_name="file.vcd")


if __name__ == '__main__':
    #unittest.main()
    plat = board.Platform()
    spi_port = plat.request("spi")
    sram = plat.request("sram")
    spi_memmapping = SpiMemmapping(spi_port, sram, 8)
    # print(verilog.convert(spi_memmapping))
    
    plat.build(spi_memmapping)





