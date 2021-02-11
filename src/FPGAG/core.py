from nmigen import Signal, Cat, Elaboratable, Record
from nmigen import Module
from nmigen.hdl.rec import DIR_FANOUT
from luna.gateware.utils.cdc import synchronize
from luna.gateware.interface.spi import SPICommandInterface, SPIBus
from luna.gateware.memory import TransactionalizedFIFO

from FPGAG.constants import COMMAND_SIZE, WORD_SIZE, STATE, START_BIT, END_BIT
from FPGAG.constants import MEMDEPTH, MEMWIDTH, COMMANDS, BYTESINGCODE


class SPIParser(Elaboratable):
    """ Parses commands over SPI """
    def __init__(self):
        self.interface = SPICommandInterface(command_size=COMMAND_SIZE,
                                             word_size=WORD_SIZE)
        self.fifo = TransactionalizedFIFO(width=MEMWIDTH,
                                          depth=MEMDEPTH)
        self.dispatcherror = Signal()  # input
        self.execute = Signal()        # output

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
        # set state
        state = Signal(COMMAND_SIZE)
        m.d.sync += [state[STATE.FULL].eq(fifo.space_available>BYTESINGCODE),
                     state[STATE.DISPATCHERROR].eq(self.dispatcherror)
        ]
        # Parser
        bytesreceived = Signal(range(BYTESINGCODE+1))
        with m.FSM(reset='RESET', name='parser'):
            with m.State('RESET'):
                m.d.sync += self.execute.eq(0)
                m.next = 'WAIT_COMMAND'
            with m.State('WAIT_COMMAND'):
                m.d.sync += [fifo.write_commit.eq(0)]
                with m.If(interface.command_ready):
                    with m.If(interface.command==COMMANDS.EMPTY):
                        m.next = 'WAIT_COMMAND'
                    with m.Elif(interface.command==COMMANDS.START):
                        m.next = 'WAIT_COMMAND'
                        m.d.sync += self.execute.eq(1)
                    with m.Elif(interface.command==COMMANDS.STOP):
                        m.next = 'WAIT_COMMAND'
                        m.d.sync += self.execute.eq(0)
                    with m.Elif(interface.command==COMMANDS.GCODE):
                        with m.If(fifo.space_available>BYTESINGCODE):
                            m.next = 'WAIT_WORD'
                        with m.Else():
                            m.next = 'WAIT_COMMAND'
                            m.d.sync += [interface.word_to_send.eq(state)]
                    with m.Elif(interface.command==COMMANDS.STATUS):
                        m.d.sync += [interface.word_to_send.eq(255)]
                        m.next = 'WAIT_COMMAND'
            with m.State('WAIT_WORD'):
                with m.If(interface.word_complete):
                    m.d.sync += [bytesreceived.eq(bytesreceived+4),
                                 fifo.write_en.eq(1),
                                 fifo.write_data.eq(interface.word_received)
                                ]
                    m.next = 'WRITE'
            with m.State('WRITE'):
                m.d.sync += [fifo.write_en.eq(0)]
                m.next = 'WAIT_COMMAND'
                with m.If(bytesreceived==BYTESINGCODE):
                    m.d.sync += [bytesreceived.eq(0),
                                 fifo.write_commit.eq(1)]
                    
        return m


class Core(Elaboratable):
    """ FPGA core for Beage G. """
    def __init__(self):
        self.spiparser = SPIParser()

    def elaborate(self, platform):
        m = Module()
        # Connect Parser
        m.submodules.spiparser = self.spiparser
        # get directions
        if platform:
            directions = platform.request("DIRECTIONS")
        else:
            directions = Record([('dirx', 1, DIR_FANOUT),
                                 ('diry', 1, DIR_FANOUT),
                                 ('dirz', 1, DIR_FANOUT)])
            self.directions = directions
        # Define dispatcher
        fifo = self.spiparser.fifo
        error = Signal()
        enabled = Signal()
        m.d.sync += [self.spiparser.dispatcherror.eq(error),
                     enabled.eq(self.spiparser.execute)]
        bytesreceived = Signal(range(BYTESINGCODE))
        with m.FSM(reset='RESET', name='dispatcher'):
            m.d.sync += [fifo.read_commit.eq(0)]
            with m.State('RESET'):
                m.next = 'WAIT_COMMAND'
            with m.State('WAIT_COMMAND'):
                with m.If((fifo.empty == 0)&(enabled==1)):
                    m.d.sync += [fifo.read_en.eq(1)]
                    m.next = 'PARSEHEAD'
            with m.State('PARSEHEAD'):
                with m.If(fifo.read_data[-8:] == COMMANDS.GCODE):
                    m.d.sync += [directions.eq(fifo.read_data[-END_BIT['DIRECTION']\
                                               :-START_BIT['DIRECTION']]),
                                 fifo.read_commit.eq(1)]
                    m.next = 'WAIT_COMMAND'
                with m.Else():
                    # NOTE: system never recovers user most reset
                    m.d.sync += [error.eq(1)]
        return m

# Overview:
#  the hardware consists out of the following elements
#  -- SPI command interface
#  -- transactionalized FIFO
#  -- SPI parser (basically an extension of SPI command interface)
#  -- Dispatcher --> dispatches signals to actual hardware


#  if you write the wrong command --> you receive an error

#   --> als ik te vaak schrijf naar bord dan raakt ie vol