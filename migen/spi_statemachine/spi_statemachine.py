"""
    spi_statemachine.py
    Example for laser scanner
    Has a lot of the complexity present in laser scanner but works with a simple LED.

    Rik Starmans
"""
from collections import namedtuple

import unittest
from migen.fhdl import verilog
from migen.fhdl.tools import list_special_ios
from migen import *
from litex.soc.cores.spi import SPIMaster, SPISlave

import sys
sys.path.append("..") 
import hexa as board

#TODO: not all these variables needed for simple example
#      reduce variable to LED blinking speed and move this to other file
# VARIABLES = {'RPM':2400,'SPINUP_TICKS':1.5,'MAX_WAIT_STABLE_TICKS':1.125, 'FACETS':4,
#             'SCANLINE_DATA_SIZE':790,'TICKS_PER_PRISM_FACET':12500, 'TICKS_START':4375,
#              'SINGLE_FACET':0, 'DIRECTION':0, 'JITTER_ALLOW':300, 'JITTER_THRESH':400}
# # lines that can be in memory
# LINES = (LEDPROGRAM.MEMWIDTH*LEDPROGRAM.MEMDEPTH)//VARIABLES['SCANLINE_DATA_SIZE']


class LEDProgram(Module):
    @staticmethod
    def commands():
        commands = ('RECVCOMMAND', 'STATUS', 'START', 'STOP', 'READ_D', 'WRITE_L')
        Commands = namedtuple('Commands', commands, defaults=tuple(range(len(commands))))
        return Commands()

    COMMANDS = commands.__func__()
    CHUNKSIZE = 8 # you write in chunks of 8 bytes
    MEMWIDTH = 8  # must be 8, as you receive in terms of eight
    

    def __init__(self, spi_port, led, memdepth=512):
        self.MEMDEPTH = memdepth
        # three submodules; SPI receiver, memory and laser state machine
        # full byte state
        ledstate   =  Signal(3)   # state laser module 6-8 byte
        error      =  Signal(4)   # error state  1-5 byte
                                  # memory full  0 byte
        
        debug = Signal(8)   # optional 

        # Memory element
        # Current idea: memory continiously repeats cycle;  idle --> read --> write
        # Rules:
        #        read cannot be set equal to write address  --> reading a line not written yet
        #        write cannot be set equal to read address  --> writing on a line which is still in use
        # Nextwrite and nextread addres used to know when writing is finished
        # writebyte, current byte written to
        # written to detect if already information is written to memory
        # sram memory is 32 blocks... each block has its own ports
        # one block 8*512 = 4096 bytes currently used
        self.specials.mem = Memory(width=self.MEMWIDTH, depth=self.MEMDEPTH, init = [10,20])
        writeport = self.mem.get_port(write_capable=True, mode = READ_FIRST)
        readport = self.mem.get_port(has_re=True)
        self.specials += writeport, readport
        self.ios = {writeport.adr, writeport.dat_w, writeport.we, readport.dat_r, readport.adr, readport.re}
        self.submodules.memory = FSM(reset_state = "RESET")
        self.nextreadaddress = Signal(max=self.MEMDEPTH)
        self.nextwriteaddress = Signal(max=self.MEMDEPTH)
        self.writebyte = Signal(max=self.MEMDEPTH)
        written = Signal()
        self.memory.act("RESET",
                NextValue(written, 0),
                NextValue(readport.adr, 0),
                NextValue(writeport.adr, self.MEMDEPTH-1),
                NextState("IDLE")
        )
        self.memory.act("IDLE",
            NextState("READ"),
            # read address set by laser scanner state machine
            NextValue(writeport.we, 0),
            NextValue(readport.re, 1)
        )
        self.memory.act("READ",
            # data read out by laser scanner state machine
            # write address and data set by spi state machine
            NextValue(self.nextreadaddress, readport.adr+1),
            NextValue(readport.re, 0),
            NextValue(writeport.we, 1),
            NextState("WRITE")
        )
        self.memory.act("WRITE",
            NextValue(self.nextwriteaddress, writeport.adr+1),
            NextValue(writeport.we, 0),
            NextState("IDLE"),
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
                NextValue(spislave.miso, Cat(ledstate+error+0)),
            If((self.nextwriteaddress==readport.adr)&(written==1),
                NextValue(spislave.miso[0],1)
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
                    NextValue(ledstate, 0)
                ).
                Elif(spislave.mosi == self.COMMANDS.START,
                    NextValue(ledstate, 1)
                ).
                Elif(spislave.mosi == self.COMMANDS.READ_D,
                    NextValue(command, self.COMMANDS.READ_D),
                    #NOTE THIS DOESN'T WORK!
                    NextValue(spislave.miso, debug)
                ).
                Elif((spislave.mosi == self.COMMANDS.WRITE_L) & (self.memoryfull == 0),
                    NextValue(command, self.COMMANDS.WRITE_L),
                    # no effect
                    #NextValue(spislave.miso, 3)
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
                    NextValue(writeport.adr, writeport.adr+1),
                    If(self.writebyte>=self.CHUNKSIZE-1,
                        NextValue(self.writebyte, 0),
                        NextValue(error[0], writeport.adr==self.nextwriteaddress),
                        NextValue(command, self.COMMANDS.RECVCOMMAND)
                    ).
                    Else(NextValue(self.writebyte, self.writebyte+1)
                    )
                )
            )
        )
        # LED State machine
        self.submodules.ledfsm = FSM(reset_state = "OFF")
        self.ledfsm.act("OFF",
            NextValue(led, 0),
            If(ledstate==1,
                 NextState("ON")
            )
        )
        self.ledfsm.act("ON",
            NextValue(led, 1),
            If(ledstate==0,
                NextState("OFF")
            )
        )


class TestSPIStateMachine(unittest.TestCase):
    def setUp(self):
        class DUT(Module):
            def __init__(self):
                pads = Record([("clk", 1), ("cs_n", 1), ("mosi", 1), ("miso", 1)])
                self.submodules.master = SPIMaster(pads, data_width=8,
                        sys_clk_freq=100e6, spi_clk_freq=5e6,
                        with_csr=False)
                self.led = Signal()
                self.submodules.ledprogram = LEDProgram(pads, self.led, memdepth=16)
        self.dut = DUT()

    def transaction(self, data_sent, data_received):
        ''' 
        helper function to test transaction from raspberry pi side
        '''
        yield self.dut.master.mosi.eq(data_sent)
        yield self.dut.master.length.eq(8)
        yield self.dut.master.start.eq(1)
        yield
        yield self.dut.master.start.eq(0)
        yield
        while (yield self.dut.master.done) == 0:
            yield
        self.assertEqual((yield self.dut.master.miso), data_received)

    def test_ledturnon(self):
        def raspberry_side():
            # get the initial status
            yield from self.transaction(LEDProgram.COMMANDS.STATUS, 0)
            # turn on the LED, status should still be zero
            yield from self.transaction(LEDProgram.COMMANDS.START, 0)
            # check wether the led is on
            yield from self.transaction(LEDProgram.COMMANDS.STATUS, 1)
            # turn OFF the led
            yield from self.transaction(LEDProgram.COMMANDS.STOP, 1)
            # LED should be off
            yield from self.transaction(LEDProgram.COMMANDS.STATUS, 0)

        def fpga_side():
            timeout = 0
            # LED should be off on the start
            self.assertEqual((yield self.dut.led), 0)
            # wait till led state changes
            while (yield self.dut.led) == 0:
                timeout += 1
                if timeout>1000:
                    raise Exception("Led doesn't turn on.")
                yield
            timeout = 0
            # LED should be on now
            self.assertEqual((yield self.dut.led), 1)
            # wait till led state changes
            while (yield self.dut.led) == 1:
                timeout += 1
                if timeout>1000:
                    raise Exception("Led doesn't turn off.")
                yield
            # LED should be off now
            self.assertEqual((yield self.dut.led), 0)
        run_simulation(self.dut, [raspberry_side(), fpga_side()])

    def test_writedata(self):
        def raspberry_side(dut):
            # write lines to memory
            for i in range(dut.ledprogram.MEMDEPTH+1):
                data_byte = i%256 # bytes can't be larger than 255
                if i%(LEDProgram.CHUNKSIZE)==0:
                    if (i>0)&((i%dut.ledprogram.MEMDEPTH)==0):
                        # check if memory is full
                        yield from self.transaction(LEDProgram.COMMANDS.WRITE_L, 1)
                        continue
                    else:
                        yield from self.transaction(LEDProgram.COMMANDS.WRITE_L, 0)
                yield from self.transaction(data_byte, 0)
            # memory is tested in litex
            in_memory = []
            loops = 10
            for i in range(loops):
                value = (yield dut.ledprogram.mem[i])
                in_memory.append(value)
            self.assertEqual(list(range(loops)),in_memory)
        run_simulation(self.dut, [raspberry_side(self.dut)])
    
    # what tests do you need?
    #   
    #   -- memory empty, can't read  --> led doesn't turn on
    #   -- memory full, can read --> up to some point




if __name__ == '__main__':
    import sys
    if len(sys.argv)>1:
        if sys.argv[1] == 'build':
            plat = board.Platform()
            spi_port = plat.request("spi")
            led = plat.request("user_led")
            spi_statemachine = LEDProgram(spi_port, led)
            plat.build(spi_statemachine, led, build_name = 'spi_statemachine')
    else:
        unittest.main()