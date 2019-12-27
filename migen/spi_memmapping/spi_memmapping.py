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


class SpiMemmapping(Module):
    def __init__(self, spi_port, sram, data_width, clk_pin):
        
        sram_dout = Signal(16)
        sram_din = Signal(16)
        sram_adr = Signal(19)
        # https://github.com/cliffordwolf/icestorm/blob/master/icefuzz/tests/sb_io.v
        self.specials += Instance("SB_IO", name = 'sram_dio [15:0]',
                            p_PIN_TYPE=0b101001,
                            p_PULLUP=0b0,
                            io_PACKAGE_PIN=sram.dat,
                            i_OUTPUT_ENABLE=sram.oe,
                            i_D_OUT_0 = sram_dout,
                            o_D_IN_0 = sram_din
                            )
        pll_lock = Signal()
        cd_pll = ClockDomain('sync')
        # https://freenode.irclog.whitequark.org/m-labs/2019-09-08
        # https://gist.github.com/cr1901/5de5b276fca539b66fe7f4493a5bfe7d
        # https://github.com/m-labs/migen/blob/master/doc/fhdl.rst
        # https://github.com/kbob/nmigen-examples/blob/master/nmigen_lib/pll.py
        self.specials += Instance("SB_PLL40_PAD", name='pll',
                            p_FEEDBACK_PATH='SIMPLE',
                            p_DELAY_ADJUSTMENT_MODE_FEEDBACK='FIXED',
                            p_DELAY_ADJUST_MODE_RELATIVE='FIXED',
                            p_PLLOUT_SELECT='GENCLK',
                            p_FDA_FEEDBACK=0b1111,
                            p_FDA_RELATIVE=0b1111,
                            p_DIVR=0b0011,
                            p_DIVF=0b0101000,
                            p_DIVQ=0b110,
                            p_FILTER_RANGE=0b010,
                            i_PACKAGE_PIN = clk_pin,
                            o_PLLOUTGLOBAL = cd_pll.clk,
                            o_LOCK = pll_lock,
                            i_BYPASS = 0b0,
                            i_RESETB = 0b1
        )

        # dit doe je altijd
        self.sync.cd_pll += sram.adr.eq(sram_adr)
        self.sync.cd_pll += sram.lb.eq(0)
        self.sync.cd_pll += sram.ub.eq(0)
        self.sync.cd_pll += sram.cs.eq(0)
        
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

if __name__ == '__main__':
    plat = board.Platform()
    spi_port = plat.request("spi")
    sram = plat.request("sram")
    clk_pin = plat.request("clk100")
    spi_memmapping = SpiMemmapping(spi_port, sram, 8, clk_pin)
    # print(verilog.convert(spi_memmapping))
    
    plat.build(spi_memmapping)





