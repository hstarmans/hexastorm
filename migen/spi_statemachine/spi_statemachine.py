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
import collections

import unittest
from migen.fhdl import verilog
from migen.fhdl.tools import list_special_ios
from migen import *
from litex.soc.cores.spi import SPIMaster, SPISlave

import sys
sys.path.append("..") 
import hexa as board


VARIABLES = {'RPM':2400,'SPINUP_TICKS':1.5,'MAX_WAIT_STABLE_TICKS':1.125,
        'FACETS':4,'SCANLINE_DATA_SIZE':790,'TICKS_PER_PRISM_FACET':12500, 'TICKS_START':4375, 'SINGLE_FACET':0,
        'DIRECTION':0, 'JITTER_ALLOW':300, 'JITTER_THRESH':400}

class SpiMemmapping(Module):
    def __init__(self, spi_port, data_width):
        # Memory element
        self.specials.mem = Memory(8, 512, init = [10,20])
        p1 = self.mem.get_port(write_capable=True, mode = READ_FIRST)
        p2 = self.mem.get_port(has_re=True)
        self.specials += p1, p2
        self.ios = {p1.adr, p1.dat_w, p1.we, p2.dat_r, p2.adr, p2.re}
        # there are three submodules; receiver, memory en laser state machine
        # receiver
        # geeft antwoorden op commandos en zet opdrachten door naar andere statemachines
        # memory
        # je geheugen is veel sneller dan je laser
        # je laat je geheugen loopen en steeds het getal van de read cursor lezen en data naar de write cursor schrijven
        # laser statemachine loopt gewoon


        # Receiver state machine
        spislave = SPISlave(spi_port, data_width)
        self.submodules.slave = spislave
        # done detector
        done_d = Signal()
        done_rise = Signal()
        self.sync += done_d.eq(spislave.done)
        self.comb += done_rise.eq(spislave.done & ~done_d)
        
        # start detector (could be refactored)
        start_d = Signal()
        start_rise = Signal()
        self.sync += start_d.eq(spislave.start)
        self.comb += start_rise.eq(spislave.start & ~start_d)
       
        
        # constants:
        SCANLINE_HEADER_SIZE = 1
        SCANLINE_DATA_SIZE = 8
        SCANLINE_ITEM_SIZE = SCANLINE_HEADER_SIZE + SCANLINE_DATA_SIZE
        #

        # STATES
        STATUS = 1
        START = 2
        STOP = 3
        READ_D = 4
        WRITE_L = 5
        READ_L = 6

        # variables associated with receiver
        self.bytecounter = Signal(max=SCANLINE_ITEM_SIZE)

        # variables associated with the laser module
        state = Signal(4)
        error = Signal(4)
        debug = Signal(8)
        memoryfull=Signal()
        towrite = Signal(SCANLINE_DATA_SIZE*8)
    
        self.submodules.receiver = FSM(reset_state = "IDLE")
        self.receiver.act("IDLE",
            If(start_rise,
                NextState("WAITFORDONE")))
        #TODO: add a timeout and fail
        self.receiver.act("WAITFORDONE",
            If(done_rise,
                NextState("PROCESSINPUT")))
        command = Signal(data_width)
        self.receiver.act("PROCESSINPUT",
            NextValue(self.bytecounter, self.bytecounter+1),
            NextState("IDLE"),
            # process command
            If(self.bytecounter == 0,
                NextValue(command, spislave.mosi),
                If(spislave.mosi == STATUS,
                    NextValue(spislave.miso, Cat(state+error))
                ).
                Elif(spislave.mosi == STOP,
                    NextValue(state, 0),
                    NextValue(spislave.miso, 0)
                ).
                Elif(spislave.mosi == START,
                    NextValue(state, 1),
                    NextValue(spislave.miso, 0)
                ).
                Elif(spislave.mosi == READ_D,
                    NextValue(spislave.miso, debug)
                ).
                Elif(spislave.mosi == WRITE_L,
                    NextValue(spislave.miso, memoryfull)
                ).
                Else(NextValue(spislave.miso,0))
            ).
            Else(
                If(command == STATUS,
                    NextValue(spislave.miso, self.bytecounter+1)
                ).
                Elif(spislave.mosi == WRITE_L,
                    If(memoryfull==1,
                        NextValue(towrite[self.bytecounter-1:self.bytecounter+7], self.bytecounter+1)
                    ),
                    NextValue(spislave.miso, 0)
                ).
                Else(NextValue(spislave.miso, 0))
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