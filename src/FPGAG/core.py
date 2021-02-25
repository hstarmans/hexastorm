from math import ceil

from nmigen import Signal, Cat, Elaboratable, Record
from nmigen import Module, Const
from nmigen.hdl.mem import Array
from nmigen.hdl.cd import ClockDomain
from nmigen.hdl.rec import DIR_FANOUT

from luna.gateware.utils.cdc import synchronize
from luna.gateware.interface.spi import SPICommandInterface, SPIBus
from luna.gateware.memory import TransactionalizedFIFO

from FPGAG.resources import StepperLayout, get_all_resources
from FPGAG.constants import COMMAND_SIZE, WORD_SIZE, STATE
from FPGAG.constants import MEMWIDTH, COMMANDS


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
    def __init__(self, platform=None):
        """ class initialization

        memdepth  -- change depth if memory, used for testing
        """ 
        if platform:
            self.platform = platform
        self.dispatcherror = Signal()
        self.execute = Signal()
        self.spi = SPIBus()
        self.read_data = Signal(MEMWIDTH)
        self.read_commit = Signal()
        self.read_en = Signal()
        self.empty = Signal()

    def elaborate(self, platform):
        m = Module()
        if platform:
            board_spi = platform.request("debug_spi")
            spi2 = synchronize(m, board_spi)
            m.d.comb  += self.spi.connect(spi2)
        if not platform:
            platform = self.platform
        spi = self.spi
        interface = SPICommandInterface(command_size=COMMAND_SIZE,
                                        word_size=WORD_SIZE)
        m.d.comb  += interface.spi.connect(spi)
        m.submodules.interface = interface 
        # Connect fifo
        fifo = TransactionalizedFIFO(width=MEMWIDTH,
                                     depth=platform.memdepth)
        if platform.name == 'Test':
            self.fifo = fifo
        m.submodules.fifo = fifo
        m.d.comb += [self.read_data.eq(fifo.read_data),
                     fifo.read_commit.eq(self.read_commit),
                     fifo.read_en.eq(self.read_en),
                     self.empty.eq(fifo.empty)]
        # set state
        state = Signal(COMMAND_SIZE) # max is actually word_size
        m.d.sync += [state[STATE.FULL].eq(fifo.space_available<ceil(platform.bytesingcode/4)),
                     state[STATE.DISPATCHERROR].eq(self.dispatcherror)
                    ]
        # Parser
        bytesreceived = Signal(range(platform.bytesingcode+1))
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
                with m.If(bytesreceived==platform.bytesingcode):
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
            steppers = [res for res in get_all_resources(platform, "steppers")]
            #aux = platform.request("AUX")
        else:
            # ideally this is done via layouts etc 
            steppers = [Record(StepperLayout())]
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
        # Motor States:  each motor has a state and a fraction used to increment it
        mstate = Array(Signal(32) for _ in range(len(steppers)))
        fraction = Array(Signal(32) for _ in range(len(steppers)))
        # Motor state is updated after certain delay
        # this delay changes as the motor increased speed
        delaycnt = Signal(32)  # threshold
        delay = Signal(32)     # current count of delay
        delaycntinit = Signal(32) # initial count must be temperorarily stored somewhere
        # used to loop through steppers when fractions are read in
        fractioncnt = Signal(range(len(steppers)))  
        with m.If(delaycnt<delay):
            m.d.cd1 += delaycnt.eq(delaycnt+1)
        with m.Else():
            m.d.cd1 += delaycnt.eq(0)
            for i in range(len(steppers)):
                m.d.cd1 += mstate[i].eq(mstate[i]+fraction[i])
        # Step Generator
        enable = Signal()
        for idx, stepper in enumerate(steppers):
            m.d.comb += stepper.step.eq(mstate[idx][-1]&enable)
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
                    # TODO: loop over steppers
                    #[directions.eq(parser.read_data[bits('DIRECTION')]),
                    #[aux.eq(parser.read_data[bits('AUX')]),
                    m.d.cd1 += [loopcnt.eq(0),
                                readtrigger.eq(~readtrigger)]
                    m.next = 'FRACTIONS'
                with m.Else():
                    # NOTE: system never recovers user must reset
                    # NOTE: also forward division error!
                    m.d.cd1 += parser.dispatcherror.eq(1)
            with m.State('FRACTIONS'):
                m.d.cd1 += readtrigger.eq(~readtrigger)  # je zou deze in een sneller clockdomein kunnen stoppen
                with m.If(fractioncnt<len(steppers)):  # TODO: check if your conter does right range
                    m.d.cd1 += fraction[fractioncnt].eq(parser.read_data)
                with m.Elif(fractioncnt==len(steppers)):
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

