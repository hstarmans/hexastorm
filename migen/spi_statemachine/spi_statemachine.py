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
    MEMWIDTH = 16
    MEMDEPTH = 256

    def __init__(self, spi_port, led):
        # three submodules; SPI receiver, memory and laser state machine
        # full byte state
        ledstate   =  Signal(3)   # state laser module
        error      =  Signal(4)   # error state
        memoryfull =  Signal()
        
        debug = Signal(8)   # optional 

        # Memory element
        # Current idea: memory continiously repeats cycle;  idle --> read --> write
        # Rules:
        #        read cannot be set equal to write address  --> reading a line not written yet
        #        write cannot be set equal to read address  --> writing on a line which is still in use
        # Nextwrite and nextread addres used to know when writing is finished
        # writebyte, current byte written to
        # sram memory is 32 blocks... each block has its own ports
        # one block 16*256 = 4096 bytes currently used
        self.specials.mem = Memory(width=self.MEMWIDTH, depth=self.MEMDEPTH, init = [10,20])
        writeport = self.mem.get_port(write_capable=True, mode = READ_FIRST)
        readport = self.mem.get_port(has_re=True)
        self.specials += writeport, readport
        self.ios = {writeport.adr, writeport.dat_w, writeport.we, readport.dat_r, readport.adr, readport.re}
        self.submodules.memory = FSM(reset_state = "IDLE")
        nextreadaddress = Signal(max=self.MEMDEPTH)
        nextwriteaddress = Signal(max=self.MEMDEPTH)
        self.writebyte = Signal(max=self.MEMWIDTH)
       
        self.comb += memoryfull.eq(nextwriteaddress==readport.adr)

        self.memory.act("IDLE",
            NextState("READ"),
            # read address set by laser scanner state machine
            NextValue(writeport.we, 0),
            NextValue(readport.re, 1)
        )
        self.memory.act("READ",
            # data read out by laser scanner state machine
            # write address and data set by spi state machine
            NextValue(nextreadaddress, readport.adr+1),
            NextValue(readport.re, 0),
            NextValue(writeport.we, 1),
            NextState("WRITE")
        )
        self.memory.act("WRITE",
            NextValue(nextwriteaddress, writeport.adr+1),
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
                NextValue(spislave.miso, Cat(ledstate+error+memoryfull)),
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
                    NextValue(spislave.miso, debug)
                ).
                Elif((spislave.mosi == self.COMMANDS.WRITE_L) & (memoryfull == 0),
                    NextValue(command, self.COMMANDS.WRITE_L),
                    NextValue(spislave.miso, 0)
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
                    # all bytes are received
                    If(self.writebyte>=self.MEMWIDTH,
                        NextValue(self.writebyte,0),
                        # If not done writing raise error
                        NextValue(error[0], writeport.adr==nextwriteaddress),
                        NextValue(writeport.adr, writeport.adr+1),
                        NextValue(command, self.COMMANDS.RECVCOMMAND)
                    ).
                    Else(
                        NextValue(writeport.dat_w[self.writebyte.variable:], spislave.mosi),
                        NextValue(self.writebyte, self.writebyte+1),
                        NextValue(spislave.miso, 0)
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
                    self.submodules.spi_statemachine = LEDProgram(pads, self.led)
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
        def raspberry_side(dut):
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

        def fpga_side(dut):
            timeout = 0
            # LED should be off on the start
            self.assertEqual((yield dut.led), 0)
            # wait till led state changes
            while (yield dut.led) == 0:
                timeout += 1
                if timeout>1000:
                    raise Exception("Led doesn't turn on.")
                yield
            timeout = 0
            # LED should be on now
            self.assertEqual((yield dut.led), 1)
            # wait till led state changes
            while (yield dut.led) == 1:
                timeout += 1
                if timeout>1000:
                    raise Exception("Led doesn't turn off.")
                yield
            # LED should be off now
            self.assertEqual((yield dut.led), 0)
        run_simulation(self.dut, [raspberry_side(self.dut), fpga_side(self.dut)])
    

    def test_writedata(self):
        def raspberry_side(dut):
            # write data to the memory
            # initiate write
            yield from self.transaction(LEDProgram.COMMANDS.WRITE_L, 0)
            # write one line of ones and zeros to memory
            for _ in range(LEDProgram.MEMWIDTH):
                data_byte = int('10101010',2)
                yield from self.transaction(data_byte, 0)
            # memory is tested in litex
            for i in range(LEDProgram.MEMWIDTH):
                value = yield dut.mem[i]
                print(value)
    # for this to work you need a modifiable memwidth and depth and freq of your led
    #
    
    # what tests do you need?
    #   -- memory full, can't write  --> you get an error when trying to write
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