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
from FPGAG.constants import (COMMAND_SIZE, WORD_SIZE, STATE,
                             MEMWIDTH, COMMANDS, BEZIER_DEGREE)


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
        """
        platform  -- used to pass test platform
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
        else:
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


class Dispatcher(Elaboratable):
    """ Instruction are buffered in SRAM
        This module reads the commands and dispatches them
        to other modules"""
    def __init__(self, platform=None):
        """
        platform  -- used to pass test platform
        """
        self.platform = platform

    def elaborate(self, platform):
        m = Module()
        if platform:
            board_spi = platform.request("debug_spi")
            spi = synchronize(m, board_spi)
            steppers = [res for res in get_all_resources(platform, "steppers")]
            aux = platform.request("AUX")
        else:
            platform = self.platform
            self.spi = SPIBus()
            spi = synchronize(m, self.spi)
            steppers = platform.steppers
            aux = platform.aux
            self.aux = aux
        # Connect Parser
        parser = SPIParser(self.platform)
        m.submodules.parser = parser
        m.d.comb  += parser.spi.connect(spi)
        # coeff for decaljau algo
        numb_coeff = platform.motors*(BEZIER_DEGREE+1)
        coeff = Array(Signal(32) for _ in range(numb_coeff))
        coeffcnt = Signal(range(numb_coeff))
        if self.platform.name == 'Test':
            self.parser = parser
            self.coeff = coeff
        with m.FSM(reset='RESET', name='dispatcher'):
            with m.State('RESET'):
                m.next = 'WAIT_COMMAND'
            with m.State('WAIT_COMMAND'):
                m.d.sync += parser.read_commit.eq(0)
                with m.If((parser.empty==0)&parser.execute):
                    m.d.sync += parser.read_en.eq(1)
                    m.next = 'PARSEHEAD'
            # check which command we r handling
            with m.State('PARSEHEAD'):
                with m.If(parser.read_data[-8:] == COMMANDS.GCODE):
                    m.d.sync += [aux.eq(parser.read_data[-16:-8]),
                                 parser.read_en.eq(0),
                                 coeffcnt.eq(0)]
                    m.next = 'BEZIERCOEFF'
                with m.Else():
                    # NOTE: system never recovers user must reset
                    m.d.sync += parser.dispatcherror.eq(1)
            with m.State('BEZIERCOEFF'):
                with m.If(parser.read_en==0):
                    m.d.sync += parser.read_en.eq(1)
                with m.Elif(coeffcnt<numb_coeff):
                    m.d.sync += [coeff[coeffcnt].eq(parser.read_data),
                                 coeffcnt.eq(coeffcnt+1),
                                 parser.read_en.eq(0)]
                # signal there is a new instruction!!
                # ideally you can keep two instruction in memory
                with m.Else():
                    m.next = 'WAIT_COMMAND'
                    m.d.sync += [parser.read_commit.eq(1),
                                 parser.read_en.eq(0)]
        return m

    
def Casteljau(Elaboratable):
    """ Sets motor states using casteljau algorithm up to second order
    """
    def __init__(self, platform=None):
        """
        platform  -- used to pass test platform
        """ 
        if platform:
            self.platform = platform
        # coeff for decaljau algo
        self.coeff = Array(Signal(32) for _ in range(len(platform.steppers)))
    
    
    def elaborate(self, platform):
        m = Module()
        with m.FSM(reset='RESET', name='dispatcher'):
            with m.State('RESET'):
                m.next = 'WAIT_COMMAND'
            with m.State('WAIT_COMMAND'):
                m.d.sync += parser.read_commit.eq(0)
                with m.If((parser.empty==0)&parser.execute):
                    m.d.sync += parser.read_en.eq(1)
                    m.next = 'PARSEHEAD'
            # check which command we r handling
            with m.State('PARSEHEAD'):
                with m.If(parser.read_en):
                    m.d.sync += parser.read_en.eq(0)
                with m.Elif(parser.read_data[-8:] == COMMANDS.GCODE):
                    m.d.sync += [aux.eq(parser.read_data[-16:-8]),
                                 parser.read_en.eq(1),
                                 steppercnt.eq(0)]
                    m.next = 'BEZIERCOEFF'
                with m.Else():
                    # NOTE: system never recovers user must reset
                    m.d.sync += parser.dispatcherror.eq(1)
            with m.State('BEZIERCOEFF'):
                with m.If(parser.read_en):
                    m.d.sync += parser.read_en.eq(0) 
                with m.Elif(steppercnt<len(steppers)):  
                    m.d.sync += [coeff[steppercnt].eq(parser.read_data),
                                 steppercnt.eq(steppercnt+1),
                                 parser.read_en.eq(1)]
                # signal there is a new instruction!!
                # ideally you can keep two instruction in memory
                with m.Else():
                    m.next = 'WAIT_COMMAND'
                    m.d.sync += parser.read_commit.eq(1)

    # combine this with cabeljau
    
#             # Motor States: each motor has a state and four bezier coefficients
#         mstate = Array(Signal(32) for _ in range(len(steppers)))
#         # Step Generator
#         enable = Signal()
#         for idx, stepper in enumerate(steppers):
#             m.d.comb += stepper.step.eq(mstate[idx][-1]&enable)
    
    
    
    
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

