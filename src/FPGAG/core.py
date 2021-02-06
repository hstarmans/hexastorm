from nmigen import Signal, Cat, Elaboratable
from nmigen import Memory, Module
from luna.gateware.utils.cdc import synchronize
from luna.gateware.interface.spi import SPICommandInterface
from luna.gateware.memory import TransactionalizedFIFO

from ..constants import COMMAND_SIZE, WORD_SIZE
from ..constants import MEMDEPTH, MEMWIDTH, COMMANDS


class Core(Elaboratable):
    """ FPGA core for Beage G. """

    def __init__(self):
        # Base ourselves around an SPI command interface.
        self.interface = SPICommandInterface(command_size=COMMAND_SIZE,
                                             word_size=WORD_SIZE,
                                             clock_phase=1)

    def elaborate(self, platform):
        m = Module()
        # Synchronize and connect SPI.
        board_spi = platform.request("debug_spi")
        spi = synchronize(m, board_spi)
        m.d.comb  += self.interface.spi.connect(spi)
        m.submodules.fifo = fifo = TransactionalizedFIFO(width=MEMWIDTH,
                                                         depth=MEMDEPTH)
        interface = self.interface
        with m.FSM(reset='RESET', name='parser'):
            with m.State('RESET'):
                m.next = 'WAIT_COMMAND'
            with m.State('WAIT_COMMAND'):
                fifo.read_commit.eq(0)
                fifo.write_commit.eq(0)
                with m.If(interface.command_ready):
                    with m.If(interface.command==COMMANDS.EMPTY):
                        m.next = 'WAIT_COMMAND'
                    with m.If(interface.command==COMMANDS.GCODE):
                        m.next = 'COMMIT_COMMAND'
                        fifo.write_data.eq(COMMANDS.GCODE)
                        fifo.write_en.eq(1)
                        fifo.read_en.eq(1)
                    with m.If(interface.command==COMMANDS.LASER):
                        m.next = 'COMMIT_COMMAND'
                        fifo.write_data.eq(COMMANDS.LASER)
                        fifo.write_en.eq(1)
                        fifo.read_en.eq(1)
                    with m.If(interface.command==COMMANDS.ABORT):
                        m.next = 'WAIT_COMMAND'
                    with m.If(interface.command==COMMANDS.EXIT):
                        m.next = 'WAIT_COMMAND'
            with m.State('COMMIT_COMMAND'):
                fifo.write_en.eq(0)
                fifo.read_en.eq(0)
                # NOTE: data read is not given back to host
                fifo.read_commit.eq(1)
                fifo.write_commit.eq(1)
                m.next = 'WAIT_WORD'
            with m.State('WAIT_WORD'):
                fifo.read_commit.eq(0)
                fifo.write_commit.eq(0)
                with m.If(interface.word_complete):
                    fifo.write_data.eq(interface.word_received)
                    fifo.write_en.eq(1)
                    fifo.read_en.eq(1)
                    m.next = 'COMMIT_WORD'
            with m.State('COMMIT_COMMAND'):
                fifo.write_en.eq(0)
                fifo.read_en.eq(0)
                fifo.write_commit.eq(1)
                fifo.read_commit.eq(1)
                interface.word_to_send.eq(fifo.read_data)
                m.next = 'WAIT_COMMAND'

# TODO:
#  1: write a test
#  2: check wether your fifo can be 40 bytes, it would allow you to pack command and word into one