from math import ceil

from nmigen import Signal, Cat, Elaboratable, Record
from nmigen import Module, Const
from nmigen.hdl.mem import Array
from nmigen.hdl.cd import ClockDomain
from nmigen.hdl.rec import DIR_FANOUT

from luna.gateware.utils.cdc import synchronize
from luna.gateware.interface.spi import SPICommandInterface, SPIBus
from luna.gateware.memory import TransactionalizedFIFO

from FPGAG.constants import COMMAND_SIZE, WORD_SIZE, bits, STATE
from FPGAG.constants import MEMDEPTH, MEMWIDTH, COMMANDS, BYTESINGCODE


class Divisor(Elaboratable):
    """ Euclidean division with a remainder
    
    X = Y*Q + R
    dividend X by divisor Y you get quotient Q and remainder R
    For a tutorial see https://projectf.io/posts/division-in-verilog/
    
        width            -- number of bits of divisor and quotients
    I/O signals:
        I: start         -- calculation starts on high
        O: busy          -- calculation in progress
        O: valid         -- result is valid
        O: dbz           -- divide by zero
        I: x             -- dividend
        I: y             -- divisor
        O: q             -- quotients
        O: r             -- remainder
    """
    def __init__(self, width=4):
        self.width = width
        self.start = Signal()
        self.busy = Signal()
        self.valid = Signal()
        self.dbz = Signal()
        self.x = Signal(width)
        self.y = Signal(width)
        self.q = Signal(width)
        self.r = Signal(width)

    def elaborate(self, platform):
        m = Module()
        ac = Signal(self.width+1)
        ac_next = Signal.like(ac)
        temp = Signal.like(ac)
        q1 = Signal(self.width)
        q1_next = Signal.like(q1)
        i = Signal(range(self.width))
        # combinatorial
        with m.If(ac>=self.y):
            m.d.comb += [temp.eq(ac-self.y),
                         Cat(q1_next, ac_next).eq(Cat(1, q1, temp[0:self.width-1]))]
        with m.Else():
            m.d.comb += [Cat(q1_next, ac_next).eq(Cat(q1, ac)<<1)]
        # synchronized
        with m.If(self.start):
            m.d.sync += [self.valid.eq(0), i.eq(0)]
            with m.If(self.y==0):
                m.d.sync += [self.busy.eq(0),
                             self.dbz.eq(1)]
            with m.Else():
                m.d.sync += [self.busy.eq(1),
                             self.dbz.eq(0),
                             Cat(q1, ac).eq(Cat(Const(0,1), self.x, Const(0, self.width)))]
        with m.Elif(self.busy):
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
        return m


class SPIParser(Elaboratable):
    """ Parses and replies to commands over SPI
    
    The following commmands are possible
      status -- send back state of the peripheriral 
      start  -- enable execution of gcode
      stop   -- halt execution of gcode
      gcode  -- write instruction to FIFO or report memory is full

    I/O signals:
        I: dispatcherror  -- error while processing stored command from spi 
        O: execute        -- start processing gcode
        I/O: Spibus       -- spi bus connected to peripheral
        O: read_data      -- read data from transactionalizedfifo
        I: read_commit    -- finalize read transactionalizedfifo
        I: read_en        -- enable read transactionalizedfifo
        O: empty          -- transactionalizedfifo is empty
    """
    def __init__(self, memdepth=None):
        """ class initialization

        memdepth  -- change depth if memory, used for testing
        """ 
        if memdepth:
            self.memdepth = memdepth
        else:
            self.memdepth = MEMDEPTH
        self.dispatcherror = Signal()
        self.execute = Signal()
        self.spi = SPIBus()
        self.read_data = Signal(MEMWIDTH)
        self.read_commit = Signal()
        self.read_en = Signal()
        self.empty = Signal()

    def elaborate(self, platform):
        m = Module()
        # Connect SPI
        #spi = synchronize(m, self.spi)  # TODO don't drive twice
        spi = self.spi
        interface = SPICommandInterface(command_size=COMMAND_SIZE,
                                        word_size=WORD_SIZE)
        m.d.comb  += interface.spi.connect(spi)
        m.submodules.interface = interface 
        # Connect fifo
        fifo = TransactionalizedFIFO(width=MEMWIDTH,
                                     depth=self.memdepth)
        if not platform:
            self.fifo = fifo # test handle
        m.submodules.fifo = fifo
        m.d.comb += [self.read_data.eq(fifo.read_data),
                     fifo.read_commit.eq(self.read_commit),
                     fifo.read_en.eq(self.read_en),
                     self.empty.eq(fifo.empty)
                    ]
        # set state
        state = Signal(COMMAND_SIZE) # max is actually word_size
        m.d.sync += [state[STATE.FULL].eq(fifo.space_available<ceil(BYTESINGCODE/4)),
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
                        with m.If((state[STATE.FULL]==0)|(bytesreceived!=0)):
                            m.next = 'WAIT_WORD'
                        with m.Else():
                            m.next = 'WAIT_COMMAND'
                            m.d.sync += [interface.word_to_send.eq(state)]
                    with m.Elif(interface.command==COMMANDS.STATUS):
                        m.d.sync += [interface.word_to_send.eq(state)]
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
    """ FPGA core for Beage G and top module """
    def __init__(self, gfreq=1):
        # TODO: this might not be clean way of creating freq
        self.count = round(100/(gfreq*2))
        if 100%(gfreq*2):
            raise Exception("Invalid, use PLL to create clock")

    def elaborate(self, platform):
        m = Module()
        # Clock domains:  Dispachter works at 1 MHz
        cd1 = ClockDomain()
        m.domains += cd1
        # TODO: you don't have reset?
        #cd1.reset = m.d['sys'].reset
        clockin = Signal()
        cd1.clock = clockin
        counter = Signal(range(self.count))
        with m.If(counter<self.count):
            m.d.sync += counter.eq(counter+1)
        with m.Else():
            m.d.sync += [counter.eq(0),
                         clockin.eq(~clockin)]
        # Directions
        if platform:
            # TODO: hack fix board with proper layou
            directions = platform.request("DIRECTIONS")
            steps = platform.request("STEPS")
            aux = platform.request("AUX")
        else:
            # ideally this is done via layouts etc 
            directions = Record([('x', 1, DIR_FANOUT),
                                 ('y', 1, DIR_FANOUT),
                                 ('z', 1, DIR_FANOUT)])
            steps = Record([('x', 1, DIR_FANOUT),
                            ('y', 1, DIR_FANOUT),
                            ('z', 1, DIR_FANOUT)])
            aux = Record([('0', 1, DIR_FANOUT)])
            self.directions = directions
        # Connect Parser
        parser = SPIParser()
        m.submodules.parser = parser
        if not platform:
            self.parser = parser
        if platform:
            board_spi = platform.request("debug_spi")
            spi = synchronize(m, board_spi)
        else:
            self.spi = SPIBus()
            spi = synchronize(m, self.spi)
        m.d.comb  += parser.spi.connect(spi)
        # Add divisor
        divisor = Divisor()
        m.submodules.divisor = divisor
        # Memory trigger
        readtrigger = Signal()
        readtrigger_d = Signal()
        m.d.sync += readtrigger_d.eq(readtrigger)
        with m.If(readtrigger!=readtrigger_d):
            m.d.comb += parser.read_en.eq(0)
        with m.Else():
            m.d.comb += parser.read_en.eq(1)
        # Motor States
        MOTORS = 3 # TODO Fix this
        delaycnt = Signal(32)
        delay = Signal(32)
        mstate = Array(Signal(32) for _ in range(MOTORS))
        fractioncnt = Signal(range(MOTORS+1))
        fraction = Array(Signal(32) for _ in range(MOTORS))
        
        with m.If(delaycnt<delay):
            m.d.cd1 += delaycnt.eq(delaycnt+1)
        with m.Else():
            m.d.cd1 += delaycnt.eq(0)
            for i in range(MOTORS):
                m.d.cd1 += mstate[i].eq(mstate[i]+fraction[i])
        # Step Generator
        delaycntinit = Signal(32)
        enable = Signal()
        # TODO: make proper loop over array
        m.d.comb += [steps.x.eq(mstate[0][-1]&enable),
                     steps.y.eq(mstate[1][-1]&enable),
                     steps.z.eq(mstate[2][-1]&enable)]
        # Define dispatcher
        loopcnt = Signal(16)
        with m.FSM(reset='RESET', name='dispatcher'):
            #m.d.cd1 += [parser.read_commit.eq(0)]
            with m.State('RESET'):
                m.next = 'WAIT_COMMAND'
            with m.State('WAIT_COMMAND'):
                with m.If((parser.empty == 0)&(parser.execute==1)):
                    m.d.cd1 += readtrigger.eq(~readtrigger)
                    m.next = 'PARSEHEAD'
            with m.State('PARSEHEAD'):
                with m.If(parser.read_data[-8:] == COMMANDS.GCODE):
                    m.d.cd1 += [directions.eq(parser.read_data[bits('DIRECTION')]),
                                aux.eq(parser.read_data[bits('AUX')]),
                                loopcnt.eq(0),
                                readtrigger.eq(~readtrigger)]
                    m.next = 'FRACTIONS'
                with m.Else():
                    # NOTE: system never recovers user must reset
                    # NOTE: also forward division error!
                    m.d.cd1 += parser.dispatcherror.eq(1)
            with m.State('FRACTIONS'):
                m.d.cd1 += readtrigger.eq(~readtrigger)  # je zou deze in een sneller clockdomein kunnen stoppen
                with m.If(fractioncnt<MOTORS):
                    m.d.cd1 += fraction[fractioncnt].eq(parser.read_data)
                with m.Elif(fractioncnt==MOTORS):
                    m.d.cd1 += delaycntinit.eq(parser.read_data)
                with m.Else():
                    m.next = 'ACCELERATE'
            with m.State('ACCELERATE'):
                with m.If(loopcnt<parser.read_data[bits('LOOPS_ACCEL')]):
                    m.d.cd1 += [divisor.x.eq(loopcnt<<1+divisor.r),
                                divisor.y.eq(loopcnt<<2+1),
                                divisor.start.eq(1),  #TODO: make this a trigger
                                loopcnt.eq(loopcnt+1)]
                with m.Else():
                    m.next = 'MOVE'
                    m.d.cd1 += loopcnt.eq(0)
            with m.State('MOVE'):
                with m.If(loopcnt<parser.read_data[bits('LOOPS_TRAVEL')]):
                    m.d.cd1 += loopcnt.eq(loopcnt+1)
                with m.Else():
                    m.next = 'DECELERATE'
                    m.d.cd1 += [readtrigger.eq(~readtrigger),
                                loopcnt.eq(0)]
            with m.State('DECELERATE'):
                with m.If(loopcnt<parser.read_data[bits('LOOPS_DECEL')]):
                    m.d.cd1 += [divisor.x.eq(loopcnt<<1+divisor.r),
                                divisor.y.eq(loopcnt<<2-1),
                                divisor.start.eq(1),
                                loopcnt.eq(loopcnt+1)]
                with m.Else():
                    m.next = 'WAIT_COMMAND'
                    m.d.cd1 += [loopcnt.eq(0),
                                readtrigger.eq(~readtrigger)]
        return m

# Overview:
#  the hardware consists out of the following elements
#  -- SPI command interface
#  -- transactionalized FIFO
#  -- SPI parser (basically an extension of SPI command interface)
#  -- Dispatcher --> dispatches signals to actual hardware

# TODO:
#   -- delay cycle shit
#   -- accel series index

# -- contemplate upon why the byte widths are what they are
#     16 bit voor loop accel  65535 --> 0.06 seconden bewegen op, done multiple times
#                                       you don't disable between loops
#   TODO:  this creates counting issue
# how to use multiple motors or select them --> this is done via the fractions
# the enable is only a safety signal for a trigger by the switches
# add readout of fractions, fix pll add
# -- add this to docs + limitation of uniform acceleration
# -- create a second circuit with a clock of 1 MHz
# -- test move
# -- write a controller class

