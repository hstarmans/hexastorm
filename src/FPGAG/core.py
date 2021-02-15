from nmigen import Signal, Cat, Elaboratable, Record
from nmigen import Module, Const
from nmigen.hdl.rec import DIR_FANOUT
from luna.gateware.utils.cdc import synchronize
from luna.gateware.interface.spi import SPICommandInterface, SPIBus
from luna.gateware.memory import TransactionalizedFIFO

from FPGAG.constants import COMMAND_SIZE, WORD_SIZE, STATE, START_BIT, END_BIT
from FPGAG.constants import MEMDEPTH, MEMWIDTH, COMMANDS, BYTESINGCODE



class Divisor(Elaboratable):
    """ Divisor

    For a tutorial see https://projectf.io/posts/division-in-verilog/
    """
    def __init__(self, width=4):
        self.width = width
        self.start = Signal()     # start signal input
        self.busy = Signal()      # calculation in progress output
        self.valid = Signal()     # quotient and remainder are valid output
        self.dbz = Signal()       # divide by zero flag output
        self.x = Signal(width)    # input
        self.y = Signal(width)    # input
        self.q = Signal(width)    # output
        self.r = Signal(width)    # output

    def elaborate(self, platform):
        m = Module()
        ac = Signal(self.width+1)
        ac_next = Signal(self.width+1)
        temp = Signal(self.width+1)
        q1 = Signal(self.width)
        q1_next = Signal(self.width)
        i = Signal(range(self.width+1))
        self.ac = ac
        self.q1 = q1
        # combinatorial
        with m.If(self.busy):
            with m.If(ac>=self.y):
                m.d.comb += [temp.eq(ac-self.y),
                             Cat(q1_next, ac_next).eq(Cat(1, q1, temp[0:self.width-1]))]
            with m.Else():
                m.d.comb += [Cat(q1_next, ac_next).eq(Cat(q1, ac)<<1)]
        with m.Else():
            m.d.comb += [q1_next.eq(0), ac_next.eq(0)]
        # synchronized
        with m.If(self.busy):
            with m.If(i == self.width-1):
                m.d.sync += [self.busy.eq(0),
                             self.valid.eq(1),
                             i.eq(0),
                             self.q.eq(q1_next),
                             self.r.eq(ac_next>>1)]
            with m.Else():
                m.d.sync += [i.eq(i+1),
                             ac.eq(ac_next),
                             q1.eq(q1_next)]
        with m.Elif(self.start):
            m.d.sync += [self.valid.eq(0),
                         i.eq(0)]
            with m.If(self.y==0):
                m.d.sync += [self.busy.eq(0),
                             self.dbz.eq(1)]
            with m.Else():
                m.d.sync += [self.busy.eq(1),
                             self.dbz.eq(0),
                             Cat(q1, ac).eq(Cat(Const(0,1), self.x, Const(0, self.width)))]
        with m.Else():
            m.d.sync += [self.q1.eq(0),
                         self.valid.eq(0),
                         i.eq(0),
                         self.ac.eq(0)]
        return m

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
                    # NOTE: system never recovers user must reset
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