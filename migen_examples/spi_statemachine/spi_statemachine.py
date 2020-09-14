from collections import namedtuple
import unittest
import sys

from migen.fhdl import verilog
from migen.fhdl.tools import list_special_ios
from migen import *
from litex.soc.cores.spi import SPISlave

import spi_statemachine_test
from hexastorm import board

class LEDProgram(Module):
    @staticmethod
    def commands():
        commands = ('RECVCOMMAND', 'STATUS', 'START', 'STOP', 'READ_D', 'WRITE_L')
        Commands = namedtuple('Commands', commands, defaults=tuple(range(len(commands))))
        return Commands()

    COMMANDS = commands.__func__()
    CHUNKSIZE = 8 # you write in chunks of 8 bytes
    MEMWIDTH = 8  # must be 8, as you receive in terms of eight
    

    def __init__(self, spi_port, led, memdepth=512, maxperiod=5):
        self.MEMDEPTH = memdepth
        self.MAXPERIOD = maxperiod
        # three submodules; SPI receiver, memory and laser state machine
        # full byte state
        self.ledstate = Signal(3)   # state laser module 6-8 byte
        self.error = Signal(4)  # error state  1-5 byte, 
                            #     -- bit 0 read error
                            # memory full  0 byte
        debug = Signal(8)   # optional 
        # Memory element
        # Current idea: memory continiously repeats cycle;  idle --> read --> write
        # Rules:
        #        read cannot be set equal to write address  --> reading a line not written yet
        #        write cannot be set equal to read address  --> writing on a line which is still in use
        # Nextwrite and nextread addres used to know when writing is finished
        # writebyte, current byte written to
        # readbit, current bit read
        # readaddress, current address read
        # written to detect if already information is written to memory
        # sram memory is 32 blocks... each block has its own ports
        # one block 8*512 = 4096 bits currently used
        self.specials.mem = Memory(width=self.MEMWIDTH, depth=self.MEMDEPTH)
        writeport = self.mem.get_port(write_capable=True, mode = READ_FIRST)
        readport = self.mem.get_port(has_re=True)
        self.specials += writeport, readport
        self.ios = {writeport.adr, writeport.dat_w, writeport.we, readport.dat_r, readport.adr, readport.re}
        self.submodules.memory = FSM(reset_state = "RESET")
        readbit = Signal(max = self.MEMWIDTH)
        self.writebyte = Signal(max=self.MEMDEPTH)
        written = Signal()
        dat_r_temp = Signal(max= self.MEMWIDTH)
        self.memory.act("RESET",
                NextValue(written, 0),
                NextValue(readport.re, 1),
                NextValue(readport.adr, 0),
                NextValue(writeport.adr, 0),
                NextState("IDLE")
        )
        # this state is useless
        self.memory.act("IDLE",
            NextState("IDLE")
        )
        # Receiver State Machine
        # Consists out of component from litex and own custom component
        # Detects whether new command is available
        spislave = SPISlave(spi_port, data_width=8)
        self.submodules.slave = spislave
        # COMMANDS 
        # The command variable contains command to be executed
        # typically the recvcommand, cannot be set externally
        command = Signal(max=len(self.COMMANDS))
        # Done detector
        done_d = Signal()
        done_rise = Signal()
        self.sync += done_d.eq(spislave.done)
        self.comb += done_rise.eq(spislave.done & ~done_d)
        # Start detector (could be refactored)
        start_d = Signal()
        start_rise = Signal()
        self.sync += start_d.eq(spislave.start)
        self.comb += start_rise.eq(spislave.start & ~start_d)
        # Custom Receiver
        self.submodules.receiver = FSM(reset_state = "IDLE")
        self.receiver.act("IDLE",
        #NOTE: simplify with cat
                NextValue(spislave.miso[1:5], self.error),
                NextValue(spislave.miso[5:], self.ledstate),
            If((writeport.adr==readport.adr)&(written==1),
                NextValue(spislave.miso[0],1)
            ).
            Else(
                NextValue(spislave.miso[0],0)
            ),
            If(start_rise,
                NextState("WAITFORDONE")
            )
        )
        self.receiver.act("WAITFORDONE",
            If(done_rise,
                NextState("PROCESSINPUT")
            )
        )
        self.receiver.act("PROCESSINPUT",
            NextState("IDLE"),
            # Read Header
            If(command == self.COMMANDS.RECVCOMMAND,
                If(spislave.mosi == self.COMMANDS.STOP,
                    NextValue(self.ledstate, 0)
                ).
                Elif(spislave.mosi == self.COMMANDS.START,
                    NextValue(self.ledstate, 1)
                ).
                Elif(spislave.mosi == self.COMMANDS.READ_D,
                    NextValue(command, self.COMMANDS.READ_D),
                    #NOTE doesn't work as you jump to idle where miso is changed
                    NextValue(spislave.miso, debug)
                ).
                Elif(spislave.mosi == self.COMMANDS.WRITE_L,
                    # only switch to write stage if memory is not full
                    If((writeport.adr==readport.adr)&(written==1),
                        NextValue(command, self.COMMANDS.RECVCOMMAND)
                    ).
                    Else(
                        NextValue(command, self.COMMANDS.WRITE_L)
                    )
                )
                # Else; Command invalid or memory full nothing happens
            ).
            # Read data after header; only applicable for debug or write line
            Else(
                If(command == self.COMMANDS.READ_D,
                    NextValue(command, self.COMMANDS.RECVCOMMAND),
                ).
                # command must be WRITE_L
                Else(
                    NextValue(written, 1),
                    NextValue(writeport.dat_w, spislave.mosi),
                    NextValue(writeport.we, 1),
                    NextState("WRITE"),
                    If(self.writebyte>=self.CHUNKSIZE-1,
                        NextValue(self.writebyte, 0),
                        NextValue(command, self.COMMANDS.RECVCOMMAND)
                    ).
                    Else(NextValue(self.writebyte, self.writebyte+1)
                    )
                )
            )
        )
        self.receiver.act("WRITE",
            NextValue(writeport.adr, writeport.adr+1), 
            NextValue(writeport.we, 0),
            NextState("IDLE")
        )
        # LED State machine
        # A led blinks every so many cycles.
        # The blink rate of the LED can be limited via a counter
        counter = Signal(16)
        #counter = Signal(max=self.MAXPERIOD.bit_length())
        self.submodules.ledfsm = FSM(reset_state = "OFF")
        self.ledfsm.act("OFF",
            NextValue(led, 0),
            NextValue(self.error[0], 0), # there is no read error 
            NextValue(readbit,0),
            If(self.ledstate==1,
                 NextState("ON")
            )
        )
        self.ledfsm.act("ON",
            If(counter == maxperiod-1,
               NextValue(counter, 0),
               # if there is no data, led off and report error
               #NOTE: would also make sense to report error if not been written yet and you try to read
               If(written==0,
                    NextState("READ"), # you nead to read again, wrong value
                    NextValue(readport.re, 0),
                    NextValue(led, 0),
                    NextValue(self.error[0], 1)
               ).
               Else(
                    NextValue(self.error[0], 0),
                    NextValue(dat_r_temp, dat_r_temp>>1),
                    NextValue(led, dat_r_temp[0]),
                    NextValue(readbit, readbit+1),
                    # you need to read again!
                    # move to next addres if end is reached
                    If(readbit==self.MEMWIDTH-1,
                        NextState("READ"), # you nead to read again, wrong value
                        NextValue(readport.re, 0),
                        NextValue(readport.adr, readport.adr+1),
                        If(readport.adr+1==writeport.adr,
                            NextValue(written,0)
                        ).
                        #NOTE: count wrap around
                        Elif((readport.adr+1==self.MEMDEPTH)&(writeport.adr==0),
                            NextValue(written,0)
                        )
                )
               )
            ).
            Else(
                NextValue(counter, counter+1)
            ),
            If(self.ledstate==0,
               NextState("OFF")
            )
        )
        self.ledfsm.act("READ",
            NextValue(counter, counter+1),
            NextValue(readport.re, 1),
            NextValue(dat_r_temp, readport.dat_r),
            NextState("ON")
        )


if __name__ == '__main__':
    if len(sys.argv)>1:
        if sys.argv[1] == 'build':
            plat = board.Platform()
            spi_port = plat.request("spi")
            led = plat.request("user_led")
            spi_statemachine = LEDProgram(spi_port, led)
            plat.build(spi_statemachine, build_name = 'spi_statemachine')
    else:
        suite = unittest.TestLoader().loadTestsFromModule(spi_statemachine_test)
        unittest.TextTestRunner(verbosity=2).run(suite)