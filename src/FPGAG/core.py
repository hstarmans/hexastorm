from math import ceil

from nmigen import Signal, Elaboratable, signed, Cat
from nmigen.build.res import ResourceError
from nmigen import Module
from nmigen.hdl.mem import Array

from luna.gateware.utils.cdc import synchronize
from luna.gateware.interface.spi import SPICommandInterface, SPIBus
from luna.gateware.memory import TransactionalizedFIFO

from FPGAG.resources import get_all_resources
from FPGAG.constants import (COMMAND_BYTES, WORD_BYTES, STATE, INSTRUCTIONS,
                             MEMWIDTH, COMMANDS, DEGREE, BIT_SHIFT,
                             MOVE_TICKS)


class SPIParser(Elaboratable):
    """ Parses and replies to commands over SPI

    The following commmands are possible
      status -- send back state of the peripheriral
      start  -- enable execution of gcode
      stop   -- halt execution of gcode
      write  -- write instruction to FIFO or report memory is full

    I/O signals:
        I/O: Spibus       -- spi bus connected to peripheral
        I: pin state      -- state of certain pins
        I: read_commit    -- finalize read transactionalizedfifo
        I: read_en        -- enable read transactionalizedfifo
        I: dispatcherror  -- error while processing stored command from spi
        O: execute        -- start processing gcode
        O: read_data      -- read data from transactionalizedfifo
        O: empty          -- transactionalizedfifo is empty
    """
    def __init__(self, platform=None, top=False):
        """
        platform  -- used to pass test platform
        """
        self.platform = platform
        self.top = top

        self.spi = SPIBus()
        self.pinstate = Signal(8)
        self.read_commit = Signal()
        self.read_en = Signal()
        self.dispatcherror = Signal()
        self.execute = Signal()
        self.read_data = Signal(MEMWIDTH)
        self.empty = Signal()

    def elaborate(self, platform):
        m = Module()
        if platform and self.top:
            board_spi = platform.request("debug_spi")
            spi2 = synchronize(m, board_spi)
            m.d.comb += self.spi.connect(spi2)
        if self.platform:
            platform = self.platform
        spi = self.spi
        interface = SPICommandInterface(command_size=COMMAND_BYTES*8,
                                        word_size=WORD_BYTES*8)
        m.d.comb += interface.spi.connect(spi)
        m.submodules.interface = interface
        # FIFO connection
        fifo = TransactionalizedFIFO(width=MEMWIDTH,
                                     depth=platform.memdepth)
        if platform.name == 'Test':
            self.fifo = fifo
        m.submodules.fifo = fifo
        m.d.comb += [self.read_data.eq(fifo.read_data),
                     fifo.read_commit.eq(self.read_commit),
                     fifo.read_en.eq(self.read_en),
                     self.empty.eq(fifo.empty)]
        # Peripheral state
        state = Signal(8)
        m.d.sync += [state[STATE.PARSING].eq(self.execute),
                     state[STATE.FULL].eq(
                     fifo.space_available <
                     ceil(platform.bytesinmove/WORD_BYTES)),
                     state[STATE.DISPATCHERROR].eq(self.dispatcherror)]
        # Parser
        bytesreceived = Signal(range(platform.bytesinmove+1))
        with m.FSM(reset='RESET', name='parser'):
            with m.State('RESET'):
                m.d.sync += self.execute.eq(0)
                m.next = 'WAIT_COMMAND'
            with m.State('WAIT_COMMAND'):
                m.d.sync += [fifo.write_commit.eq(0)]
                with m.If(interface.command_ready):
                    with m.If(interface.command == COMMANDS.EMPTY):
                        m.next = 'WAIT_COMMAND'
                    with m.Elif(interface.command == COMMANDS.START):
                        m.next = 'WAIT_COMMAND'
                        m.d.sync += self.execute.eq(1)
                    with m.Elif(interface.command == COMMANDS.STOP):
                        m.next = 'WAIT_COMMAND'
                        m.d.sync += self.execute.eq(0)
                    with m.Elif(interface.command == COMMANDS.WRITE):
                        with m.If((state[STATE.FULL] == 0) |
                                  (bytesreceived != 0)):
                            m.next = 'WAIT_WORD'
                        with m.Else():
                            # ensure memfull only passed if not accepted
                            # memfull is already triggered during the last
                            # which fills the memory
                            m.d.sync += interface.word_to_send.eq(state)
                            m.next = 'WAIT_COMMAND'
                    with m.Elif(interface.command == COMMANDS.READ):
                        m.d.sync += interface.word_to_send.eq(Cat(state,
                                                                  self.pinstate
                                                                  ))
                        m.next = 'WAIT_COMMAND'
            with m.State('WAIT_WORD'):
                with m.If(interface.word_complete):
                    m.d.sync += [bytesreceived.eq(bytesreceived+WORD_BYTES),
                                 fifo.write_en.eq(1),
                                 fifo.write_data.eq(interface.word_received)]
                    m.next = 'WRITE'
            with m.State('WRITE'):
                m.d.sync += fifo.write_en.eq(0)
                m.next = 'WAIT_COMMAND'
                with m.If(bytesreceived == platform.bytesinmove):
                    m.d.sync += [bytesreceived.eq(0),
                                 fifo.write_commit.eq(1)]
        return m


class Polynomal(Elaboratable):
    """ Sets motor states using a polynomal algorithm

        A polynomal up to 3 order, e.g. c*x^3+b*x^2+a*x,
        is evaluated using the assumption that x starts at 0
        and y starts at 0. The polynomal determines the stepper
        position. The bitshift bit determines
        the position. In every tick the step can at most increase
        with one count.

        I/O signals:
        I: coeff          -- polynomal coefficients
        I: start          -- start signal
        O: busy           -- busy signal
        O: finished       -- finished signal
        O: total steps    -- total steps executed in move
        O: dir            -- direction; 1 is postive and 0 is negative
        O: step           -- step signal
    """
    def __init__(self, platform=None):
        self.platform = platform
        self.order = DEGREE
        # change code for other orders
        assert self.order == 3
        self.motors = platform.motors
        self.max_steps = int(MOVE_TICKS/2)  # Nyquist
        # inputs
        self.coeff = Array()
        for _ in range(self.motors):
            self.coeff.extend([Signal(signed(64)),
                               Signal(signed(64)),
                               Signal(signed(64))])
        self.start = Signal()
        self.ticklimit = Signal(MOVE_TICKS.bit_length())
        # output
        self.busy = Signal()
        self.finished = Signal()
        self.totalsteps = Array(Signal(signed(self.max_steps.bit_length()+1))
                                for _ in range(self.motors))
        self.dir = Array(Signal() for _ in range(self.motors))
        self.step = Array(Signal() for _ in range(self.motors))

    def elaborate(self, platform):
        m = Module()
        # pos
        max_bits = (self.max_steps << BIT_SHIFT).bit_length()
        cntrs = Array(Signal(signed(max_bits+1))
                      for _ in range(len(self.coeff)))
        assert max_bits <= 64
        ticks = Signal(MOVE_TICKS.bit_length())
        if platform:
            steppers = [res for res in get_all_resources(platform, "stepper")]
        else:
            steppers = self.platform.steppers
            self.ticks = ticks
            self.cntrs = cntrs
        for idx, stepper in enumerate(steppers):
            m.d.comb += [stepper.step.eq(self.step[idx]),
                         stepper.dir.eq(self.dir[idx])]
        # steps
        for motor in range(self.motors):
            m.d.comb += [self.step[motor].eq(
                         cntrs[motor*self.order][BIT_SHIFT]),
                         self.totalsteps[motor].eq(
                         cntrs[motor*self.order] >> (BIT_SHIFT+1))]
        # directions
        counter_d = Array(Signal(signed(max_bits+1))
                          for _ in range(self.motors))
        for motor in range(self.motors):
            m.d.sync += counter_d[motor].eq(cntrs[motor*self.order])
            # negative case --> decreasing
            with m.If(counter_d[motor] > cntrs[motor*self.order]):
                m.d.sync += self.dir[motor].eq(0)
            # positive case --> increasing
            with m.Elif(counter_d[motor] < cntrs[motor*self.order]):
                m.d.sync += self.dir[motor].eq(1)
        with m.FSM(reset='RESET', name='polynomen'):
            with m.State('RESET'):
                m.next = 'WAIT_START'
                m.d.sync += [self.busy.eq(0),
                             self.finished.eq(0)]
            with m.State('WAIT_START'):
                with m.If(self.start):
                    for motor in range(self.motors):
                        m.d.sync += [cntrs[motor*self.order].eq(0),
                                     counter_d[motor].eq(0)]
                    m.d.sync += [self.busy.eq(1),
                                 self.finished.eq(0)]
                    m.next = 'RUNNING'
            with m.State('RUNNING'):
                with m.If(ticks < self.ticklimit):
                    m.d.sync += ticks.eq(ticks+1)
                    for motor in range(self.motors):
                        start = motor*self.order
                        op3 = 3*2*self.coeff[start+2] + cntrs[start+2]
                        op2 = (cntrs[start+2] + 2*self.coeff[start+1]
                               + cntrs[start+1])
                        op1 = (self.coeff[start+2] + self.coeff[start+1]
                               + self.coeff[start] + cntrs[start+2] +
                               cntrs[start+1] + cntrs[start])
                        m.d.sync += [cntrs[start+2].eq(op3),
                                     cntrs[start+1].eq(op2),
                                     cntrs[start].eq(op1)]
                with m.Else():
                    m.d.sync += [ticks.eq(0),
                                 self.busy.eq(0),
                                 self.finished.eq(1)]
                    m.next = 'WAIT_START'
        return m


class Dispatcher(Elaboratable):
    """ Dispatches instructions to right submodule

        Instructions are buffered in SRAM. This module checks the buffer
        and dispatches the instructions to the corresponding module.
        This is the top module"""
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
            try:
                aux = platform.request("AUX")
            except ResourceError:
                aux = None
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
        m.d.comb += parser.spi.connect(spi)
        # Connect Polynomal Move module
        polynomal = Polynomal(self.platform)
        m.submodules.polynomal = polynomal
        coeffcnt = Signal(2)
        # Busy signal
        busy = Signal()
        m.d.comb += busy.eq(polynomal.busy)
        if platform.name == 'Test':
            self.parser = parser
            self.coeff = polynomal.coeff
        with m.FSM(reset='RESET', name='dispatcher'):
            with m.State('RESET'):
                m.next = 'WAIT_INSTRUCTION'
            with m.State('WAIT_INSTRUCTION'):
                m.d.sync += [parser.read_commit.eq(0), polynomal.start.eq(0)]
                with m.If((parser.empty == 0) & parser.execute):
                    m.d.sync += parser.read_en.eq(1)
                    m.next = 'PARSEHEAD'
            # check which instruction we r handling
            with m.State('PARSEHEAD'):
                with m.If(parser.read_data[:8] == INSTRUCTIONS.MOVE):
                    # NOTE: add ticks here
                    # if aux is not None:
                    #    m.d.sync += aux.eq(parser.read_data[8:16])
                    m.d.sync += [parser.read_en.eq(0),
                                 coeffcnt.eq(0)]
                    m.next = 'MOVE_POLYNOMAL'
                with m.Else():
                    m.next = 'ERROR'
                    m.d.sync += parser.dispatcherror.eq(1)
            with m.State('MOVE_POLYNOMAL'):
                with m.If(parser.read_en == 0):
                    m.d.sync += parser.read_en.eq(1)
                with m.Elif(busy):
                    m.next = 'MOVE_POLYNOMAL'
                with m.Elif(coeffcnt < len(polynomal.coeff)):
                    m.d.sync += [polynomal.coeff[coeffcnt].eq(
                                 parser.read_data),
                                 coeffcnt.eq(coeffcnt+1),
                                 parser.read_en.eq(0)]
                with m.Else():
                    m.next = 'WAIT_INSTRUCTION'
                    m.d.sync += [polynomal.start.eq(1),
                                 parser.read_commit.eq(1),
                                 parser.read_en.eq(0)]
            # NOTE: system never recovers user must reset
            with m.State('ERROR'):
                m.next = 'ERROR'
        return m


# Overview:
#  the hardware consists out of the following elements
#  -- SPI command interface
#  -- transactionalized FIFO
#  -- SPI parser (basically an extension of SPI command interface)
#  -- Dispatcher --> dispatches signals to actual hardware
#  -- Polynomal integrator --> determines position via integrating polynomen

# TODO:
#   -- connect modules & test
#   -- move test and check count
#   -- blocking behaviour during move
#   -- homing test
#   -- motor should be updated with certain freq
#   -- build test
