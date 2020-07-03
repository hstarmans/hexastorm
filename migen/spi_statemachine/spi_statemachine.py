"""
    spi_statemachine.py
    Example for laser scanner
    Has a lot of the complexity present in laser scanner but works with a simple LED.

    Rik Starmans
"""
import collections

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

VARIABLES = {'RPM':2400,'SPINUP_TICKS':1.5,'MAX_WAIT_STABLE_TICKS':1.125, 'FACETS':4,
            'SCANLINE_DATA_SIZE':790,'TICKS_PER_PRISM_FACET':12500, 'TICKS_START':4375,
             'SINGLE_FACET':0, 'DIRECTION':0, 'JITTER_ALLOW':300, 'JITTER_THRESH':400}


class SpiStateMachine(Module):
    def __init__(self, spi_port, data_width):
        # three submodules; SPI receiver, memory and laser state machine

        # full byte state
        state      =  Signal(3)   # state laser module
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
        # sram memory is 32 blocks... each block has to be handled seperately
        # one block 16*256 = 4096 bytes
        WIDTH = 16
        DEPTH = 256
        LINES = (WIDTH*DEPTH)//VARIABLES['SCANLINE_DATA_SIZE']
        self.specials.mem = Memory(width=WIDTH, depth=DEPTH, init = [10,20])
        p1 = self.mem.get_port(write_capable=True, mode = READ_FIRST)
        p2 = self.mem.get_port(has_re=True)
        self.specials += p1, p2
        self.ios = {p1.adr, p1.dat_w, p1.we, p2.dat_r, p2.adr, p2.re}
        self.submodules.memory = FSM(reset_state = "IDLE")
        nextreadaddress = Signal(max=DEPTH)
        nextwriteaddress = Signal(max=DEPTH)
        self.writebyte = Signal(max=WIDTH)
       
        self.comb = memoryfull.eq(nextwriteaddress==p2.adr)

        self.memory.act("IDLE",
            NextState("READ"),
            # read address set by laser scanner state machine
            NextValue(p1.we, 0),
            NextValue(p2.re, 1)
        )
        self.memory.act("READ",
            # data read out by laser scanner state machine
            # write address and data set by spi state machine
            NextValue(nextreadaddress, p2.adr+1),
            NextValue(p2.re, 0),
            NextValue(p1.we, 1),
            NextState("WRITE")
        )
        self.memory.act("WRITE",
            NextValue(nextwriteaddress, p1.adr+1),
            NextValue(p1.we, 0),
            NextState("IDLE"),
        )
        # Receiver State Machine
        # Consists out of component from litex and own custom component
        # Detects wether new command is available
        spislave = SPISlave(spi_port, data_width)
        self.submodules.slave = spislave

        # COMMANDS  --> TODO: could be moved to DICT
        STATUS = 1
        START = 2
        STOP = 3
        READ_D = 4
        WRITE_L = 5
        READ_L = 6
        # The command variable contains command which will be executed
        # Often this is the recvcommand, cannot be set externally
        RECVCOMMAND = 0
        command = Signal(max=6) # max is READ_L
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
            NextValue(spislave.miso, Cat(state+error+memoryfull)),
            If(start_rise,
                NextState("WAITFORDONE")))
        self.receiver.act("WAITFORDONE",
            If(done_rise,
                NextState("PROCESSINPUT")))
        self.receiver.act("PROCESSINPUT",
            NextState("IDLE"),
            # Read Header
            If(command == RECVCOMMAND,
                If(spislave.mosi == STOP,
                    NextValue(state, 0)
                ).
                Elif(spislave.mosi == START,
                    NextValue(state, 1)
                ).
                Elif(spislave.mosi == READ_D,
                    NextValue(command, READ_D),
                    NextValue(spislave.miso, debug)
                ).
                Elif(spislave.mosi == WRITE_L & memoryfull == 0,
                    NextValue(command, WRITE_L),
                    NextValue(spislave.miso, 0)
                )
            ).
            # Read data after header; only applicable for debug or write line
            Else(
                If(command == READ_D,
                    NextValue(command, RECVCOMMAND),
                ).
                # command must be WRITE_L
                Else(
                    # all bytes are received
                    If(self.writebyte>=WIDTH,
                        NextValue(self.writebyte,0),
                        # Not done writing, so can't move to next address, raise error
                        NextValue(error[0], p1.adr==nextwriteaddress),
                        NextValue(p1.adr, p1.adr+1),
                        NextValue(command, RECVCOMMAND)
                    ).
                    Else(
                        NextValue(p1.dat_w[self.writebyte:], spislave.mosi),
                        NextValue(self.writebyte, self.writebyte+1),
                        NextValue(spislave.miso, 0)
                    )
                )
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
            self.assertEqual((yield dut.spimemmap.writebyte), 0)
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