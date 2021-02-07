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
        # Base ourselves around an SPI command interface.
        self.interface = SPICommandInterface(command_size=COMMAND_SIZE,
                                             word_size=WORD_SIZE)
        self.fifo = TransactionalizedFIFO(width=MEMWIDTH,
                                          depth=MEMDEPTH)
        self.led = Signal()
    def elaborate(self, platform):
        m = Module()
        # Synchronize and connect SPI.
        if platform:
            board_spi = platform.request("debug_spi")
            spi = synchronize(m, board_spi)
        else:
            self.spi = SPIBus() # needed for test
            spi = synchronize(m, self.spi)
        m.d.comb  += self.interface.spi.connect(spi)
        m.submodules.fifo = fifo = self.fifo
        # Process commands from SPI interface
        interface = self.interface
        with m.FSM(reset='RESET') as parser:
            with m.State('RESET'):
                self.led.eq(1)
                m.next = 'WAIT_COMMAND'
            with m.State('WAIT_COMMAND'):
                self.led.eq(1)
                fifo.write_commit.eq(0)
                with m.If(interface.command_ready):
                    with m.If(interface.command==COMMANDS.EMPTY):
                        m.next = 'WAIT_COMMAND'
                    with m.If((interface.command==COMMANDS.GCODE) &
                               (fifo.space_available>0)):
                        fifo.write_en.eq(1)
                        m.next = 'WAIT_WORD'
                    with m.If(interface.command==COMMANDS.ABORT):
                        m.next = 'WAIT_COMMAND'
                    with m.If(interface.command==COMMANDS.EXIT):
                        m.next = 'WAIT_COMMAND'
                    # set current state to interface
                    interface.command.eq(0)
            with m.State('WAIT_WORD'):
                with m.If(interface.word_complete):
                    fifo.write_data.eq(interface.word_received)
                    m.next = 'COMMIT_WORD'
            with m.State('COMMIT_WORD'):
                fifo.write_en.eq(0)
                print('ik benhier')
                # NOTE: we don't sent data back
                # interface.word_to_send.eq(
                m.next = 'FINALIZE'
            with m.State('FINALIZE'):
                fifo.write_commit.eq(1)
                m.next = 'WAIT_COMMAND'
        return m

# TODO:
#  1: write a test
#   --> als ik schrijf naar het bord met GCODE dan verandert de buffer
#   --> als ik te vaak schrijf naar bord dan raakt ie vol
#  2: check wether your fifo can be 40 bytes, it would allow you to pack command and word into one