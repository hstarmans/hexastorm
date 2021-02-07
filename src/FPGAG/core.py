from nmigen import Signal, Cat, Elaboratable
from nmigen import Module
from luna.gateware.utils.cdc import synchronize
from luna.gateware.interface.spi import SPICommandInterface, SPIBus
from luna.gateware.memory import TransactionalizedFIFO

from FPGAG.constants import COMMAND_SIZE, WORD_SIZE
from FPGAG.constants import MEMDEPTH, MEMWIDTH, COMMANDS


class Core(Elaboratable):
    """ FPGA core for Beage G. """

    def __init__(self):
        self.interface = SPICommandInterface(command_size=COMMAND_SIZE,
                                             word_size=WORD_SIZE)
        self.fifo = TransactionalizedFIFO(width=MEMWIDTH,
                                          depth=MEMDEPTH)
    def elaborate(self, platform):
        m = Module()
        # Synchronize and connect SPI.
        if platform:
            board_spi = platform.request("debug_spi")
            spi = synchronize(m, board_spi)
        else:
            self.spi = SPIBus()
            spi = synchronize(m, self.spi)
        m.d.comb  += self.interface.spi.connect(spi)
        m.submodules.interface = interface = self.interface
        # Connect fifo
        m.submodules.fifo = fifo = self.fifo
        with m.FSM(reset='RESET', name='parser'):
            with m.State('RESET'):
                m.next = 'WAIT_COMMAND'
            with m.State('WAIT_COMMAND'):
                m.d.sync += [fifo.write_commit.eq(0)]
                with m.If(interface.command_ready):
                    with m.If(interface.command==COMMANDS.EMPTY):
                        m.next = 'WAIT_COMMAND'
                    with m.If((interface.command==COMMANDS.GCODE) &
                               (fifo.space_available>0)):
                        m.next = 'WAIT_WORD'
                    with m.If(interface.command==COMMANDS.ABORT):
                        m.next = 'WAIT_COMMAND'
                    with m.If(interface.command==COMMANDS.EXIT):
                        m.next = 'WAIT_COMMAND'
                    # TODO: set current state to interface
                    #m.d.sync += interface.command.eq(0)
            with m.State('WAIT_WORD'):
                with m.If(interface.word_complete):
                    m.d.sync += [fifo.write_en.eq(1),
                                 fifo.write_data.eq(interface.word_received)
                                ]
                    m.next = 'WRITE'
            with m.State('WRITE'):
                m.d.sync += [fifo.write_en.eq(0),
                             fifo.write_commit.eq(1)]
                m.next = 'WAIT_COMMAND'
        return m

# TODO:
#  1: write a test
#   --> als ik te vaak schrijf naar bord dan raakt ie vol
#  2: check wether you can synthesize memory