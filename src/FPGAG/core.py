from nmigen import Signal, Elaboratable, signed, Cat
from nmigen import Module
from nmigen.hdl.mem import Array

from luna.gateware.utils.cdc import synchronize
from luna.gateware.interf.spi import SPICommandInterface, SPIBus
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
        I: positions      -- positions of stepper motors
        I: pin state      -- state of certain pins
        I: read_commit    -- finalize read transactionalizedfifo
        I: read_en        -- enable read transactionalizedfifo
        I: dispatcherror  -- error while processing stored command from spi
        O: execute        -- start processing gcode
        O: read_data      -- read data from transactionalizedfifo
        O: empty          -- transactionalizedfifo is empty
    """
    def __init__(self, platform, top=False):
        """
        platform  -- pass test platform
        top       -- trigger synthesis of module
        """
        self.platform = platform
        self.top = top

        self.spi = SPIBus()
        self.position = Array(Signal(signed(64))
                              for _ in range(platform.motors))
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
        interf = SPICommandInterface(command_size=COMMAND_BYTES*8,
                                     word_size=WORD_BYTES*8)
        m.d.comb += interf.spi.connect(spi)
        m.submodules.interf = interf
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
        # Parser
        mtrcntr = Signal(range(platform.motors))
        wordsreceived = Signal(range(platform.wordsinmove+1))
        error = Signal()
        # Peripheral state
        state = Signal(8)
        m.d.sync += [state[STATE.PARSING].eq(self.execute),
                     state[STATE.FULL].eq(fifo.space_available <= 1),
                     state[STATE.ERROR].eq(self.dispatcherror | error)]
        # remember which word we are processing
        instruction = Signal(8)
        with m.FSM(reset='RESET', name='parser'):
            with m.State('RESET'):
                m.d.sync += [self.execute.eq(0), wordsreceived.eq(0),
                             error.eq(0)]
                m.next = 'WAIT_COMMAND'
            with m.State('WAIT_COMMAND'):
                with m.If(interf.command_ready):
                    word = Cat(state[::-1], self.pinstate[::-1])
                    with m.If(interf.command == COMMANDS.EMPTY):
                        m.next = 'WAIT_COMMAND'
                    with m.Elif(interf.command == COMMANDS.START):
                        m.next = 'WAIT_COMMAND'
                        m.d.sync += self.execute.eq(1)
                    with m.Elif(interf.command == COMMANDS.STOP):
                        m.next = 'WAIT_COMMAND'
                        m.d.sync += self.execute.eq(0)
                    with m.Elif(interf.command == COMMANDS.WRITE):
                        m.d.sync += interf.word_to_send.eq(word)
                        with m.If(state[STATE.FULL] == 0):
                            m.next = 'WAIT_WORD'
                        with m.Else():
                            m.next = 'WAIT_COMMAND'
                    with m.Elif(interf.command == COMMANDS.READ):
                        m.d.sync += interf.word_to_send.eq(word)
                        m.next = 'WAIT_COMMAND'
                    with m.Elif(interf.command == COMMANDS.POSITION):
                        # position is requested multiple times for multiple
                        # motors
                        with m.If(mtrcntr < platform.motors):
                            m.d.sync += mtrcntr.eq(mtrcntr+1)
                        with m.Else():
                            m.d.sync += mtrcntr.eq(0)
                        m.d.sync += interf.word_to_send.eq(
                                                self.position[mtrcntr])
                        m.next = 'WAIT_COMMAND'
            with m.State('WAIT_WORD'):
                with m.If(interf.word_complete):
                    byte0 = interf.word_received[:8]
                    with m.If(wordsreceived == 0):
                        with m.If((byte0 == INSTRUCTIONS.MOVE) |
                                  (byte0 == INSTRUCTIONS.WRITEPIN)):
                            m.d.sync += [instruction.eq(byte0),
                                         fifo.write_en.eq(1),
                                         wordsreceived.eq(wordsreceived+1),
                                         fifo.write_data.eq(
                                             interf.word_received)]
                            m.next = 'WRITE'
                        with m.Else():
                            m.d.sync += error.eq(1)
                            m.next = 'WAIT_COMMAND'
                    with m.Else():
                        m.d.sync += [fifo.write_en.eq(1),
                                     wordsreceived.eq(wordsreceived+1),
                                     fifo.write_data.eq(interf.word_received)]
                        m.next = 'WRITE'
            with m.State('WRITE'):
                m.d.sync += fifo.write_en.eq(0)
                with m.If(((instruction == INSTRUCTIONS.MOVE) &
                          (wordsreceived >= platform.wordsinmove))
                          | (instruction == INSTRUCTIONS.WRITEPIN)):
                    m.d.sync += [wordsreceived.eq(0),
                                 fifo.write_commit.eq(1)]
                    m.next = 'COMMIT'
                with m.Else():
                    m.next = 'WAIT_COMMAND'
            with m.State('COMMIT'):
                m.d.sync += fifo.write_commit.eq(0)
                m.next = 'WAIT_COMMAND'
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
    def __init__(self, platform=None, divider=50, top=False):
        '''
            platform  -- pass test platform
            divider -- original clock of 100 MHz via PLL reduced to 50 MHz
                       if this is divided by 50 motor state updated
                       with 1 Mhz
            top       -- trigger synthesis of module
        '''
        self.top = top
        self.divider = divider
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
        self.totalsteps = Array(Signal(signed(self.max_steps.bit_length()+1))
                                for _ in range(self.motors))
        self.dir = Array(Signal() for _ in range(self.motors))
        self.step = Array(Signal() for _ in range(self.motors))

    def elaborate(self, platform):
        m = Module()
        # add 1 MHZ clock domain
        cntr = Signal(range(self.divider))
        # pos
        max_bits = (self.max_steps << BIT_SHIFT).bit_length()
        cntrs = Array(Signal(signed(max_bits+1))
                      for _ in range(len(self.coeff)))
        assert max_bits <= 64
        ticks = Signal(MOVE_TICKS.bit_length())
        if self.top:
            steppers = [res for res in get_all_resources(platform, "stepper")]
            assert len(steppers) != 0
            for idx, stepper in enumerate(steppers):
                m.d.comb += [stepper.step.eq(self.step[idx]),
                             stepper.dir.eq(self.dir[idx])]
        else:
            self.ticks = ticks
            self.cntrs = cntrs

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
                m.d.sync += self.busy.eq(0)
            with m.State('WAIT_START'):
                m.d.sync += self.busy.eq(0)
                with m.If(self.start):
                    for motor in range(self.motors):
                        coef0 = motor*self.order
                        m.d.sync += [cntrs[coef0+2].eq(0),
                                     cntrs[coef0+1].eq(0),
                                     cntrs[coef0].eq(0),
                                     counter_d[motor].eq(0)]
                    m.d.sync += self.busy.eq(1)
                    m.next = 'RUNNING'
            with m.State('RUNNING'):
                with m.If((ticks < self.ticklimit) & (cntr >= self.divider-1)):
                    m.d.sync += [ticks.eq(ticks+1),
                                 cntr.eq(0)]
                    for motor in range(self.motors):
                        idx = motor*self.order
                        op3 = 3*2*self.coeff[idx+2] + cntrs[idx+2]
                        op2 = (cntrs[idx+2] + 2*self.coeff[idx+1]
                               + cntrs[idx+1])
                        op1 = (self.coeff[idx+2] + self.coeff[idx+1]
                               + self.coeff[idx] + cntrs[idx+2] +
                               cntrs[idx+1] + cntrs[idx])
                        m.d.sync += [cntrs[idx+2].eq(op3),
                                     cntrs[idx+1].eq(op2),
                                     cntrs[idx].eq(op1)]
                with m.Elif(ticks < self.ticklimit):
                    m.d.sync += cntr.eq(cntr+1)
                with m.Else():
                    m.d.sync += ticks.eq(0)
                    m.next = 'WAIT_START'
        return m


class Laserhead(Elaboratable):
    """ Controller of laser scanner with rotating mirror or prism

        I/O signals:
        O: synchronized   -- if true, laser is in sync and prism is rotating
        O: error          -- error signal
        O: lasers         -- laser pin
        O: pwm            -- pulse for scanner motor
        O: enablepin      -- enable pin scanner motor
        I: synchronize    -- activate synchorinzation
        I: photodiode     -- trigger for photodiode
        O: read_commit    -- finalize read transactionalizedfifo
        O: read_en        -- enable read transactionalizedfifo
        I: read_data      -- read data from transactionalizedfifo
        I: empty          -- signal wether fifo is empty
    """
    def __init__(self, platform=None, divider=50, top=False):
        '''
        top        -- trigger synthesis of module
        platform   -- pass test platform
        divider    -- original clock of 100 MHz via PLL reduced to 50 MHz
                      if this is divided by 50 laser state updated
                      with 1 MHz
        '''
        self.divider = divider
        self.status = Signal()
        self.lasers = Signal(2)
        self.pwm = Signal()
        self.enablepin = Signal()
        self.synchronize = Signal()
        self.error = Signal()
        self.photodiode = Signal()
        self.read_commit = Signal()
        self.read_en = Signal()
        self.read_data = Signal()
        self.empty = Signal()

    def elaborate(self, platform):
        m = Module()
        if self.platform is not None:
            platform = self.platform
        var = platform.laserhead
        # parameter creation
        ticksinfacet = round(var['CRYSTAL_HZ']/(var['RPM']/60*var['FACETS']))
        laserticks = int(var['CRYSTAL_HZ']/var['LASER_HZ'])
        jitterticks = round(0.5*laserticks)
        if var['END%'] > round(1-(self.JITTERTICKS+1)
                               / self.ticksinfacet):
            raise Exception("Invalid settings, END% too high")
        bitsinscanline = round((self.ticksinfacet*(var['END%']-var['START%']))
                               / laserticks)
        if bitsinscanline <= 0:
            raise Exception("Bits in scanline invalid")
        # Pulse generator for prism motor
        polyperiod = int(var['CRYSTAL_HZ']/(var['RPM']/60)/(6*2))
        pwmcnt = Signal(range(polyperiod))
        poly_en = Signal()
        # pwm is always created but can be deactivated
        with m.If(pwmcnt == 0):
            m.d.sync += [self.pwm.eq(~self.pwm),
                         pwmcnt.eq(polyperiod-1)]
        with m.Else():
            m.d.sync += pwmcnt.eq(pwmcnt+1)
        # Laser FSM
        facetcnt = Signal(max=var['FACETS'])
        spinupticks = round(var['SPINUP_TIME']*var['CRYSTAL_HZ'])
        stableticks = round(var['STABLE_TIME']*var['CRYSTAL_HZ'])
        stablecntr = Signal(max=max(spinupticks, stableticks))
        stablethresh = Signal(range(stableticks))
        lasercnt = Signal(range(laserticks))
        scanbit = Signal(range(bitsinscanline+1))
        tickcounter = Signal(range(self.ticksinfacet*2))
        photodiode = self.photodiode
        read_data = self.read_data
        read_old = Signal.like(read_data)
        readbit = Signal(range(MEMWIDTH))
        photodiode_d = Signal()
        lasers = self.lasers
        with m.FSM(reset='RESET', name='laserfsm'):
            with m.State('RESET'):
                m.d.sync += self.error.eq(0)
                m.next = 'STOP'
            with m.State('STOP'):
                m.d.sync += [stablethresh.eq(stableticks-1),
                             stablecntr.eq(0),
                             poly_en.eq(0),
                             readbit.eq(0),
                             facetcnt.eq(0),
                             scanbit.eq(0),
                             lasercnt.eq(0),
                             lasers.eq(0)]
                with m.If(self.synchronize):
                    # laser is off, photodiode cannot be triggered
                    # TODO: add check that fifo is not empty
                    with m.If(self.photodiode == 0):
                        m.d.sync += self.error.eq(1)
                        m.next = 'STOP'
                    with m.Else():
                        m.d.sync += [self.error.eq(0),
                                     poly_en.eq(1)]
                        m.next = 'SPINUP'
            with m.State('SPINUP'):
                with m.If(stablecntr > spinupticks-1):
                    # turn on laser
                    m.d.sync += [self.lasers.eq(int('1'*2, 2)),
                                 stablecntr.eq(0)]
                    m.next = 'WAIT_STABLE'
                with m.Else():
                    m.d.sync += stablecntr.eq(stablecntr+1)
            with m.State('WAIT_STABLE'):
                m.d.sync += [stablecntr.eq(stablecntr+1),
                             photodiode_d.eq(photodiode)]
                with m.If(stablecntr > stablethresh):
                    m.d.sync += self.error.eq(1)
                    m.next = 'STOP'
                with m.Elif(~photodiode & ~photodiode_d):
                    m.d.sync += [tickcounter.eq(0),
                                 lasers.eq(0)]
                    with m.If((tickcounter > ticksinfacet-jitterticks) &
                              (tickcounter < ticksinfacet+jitterticks)):
                        with m.If(facetcnt == var['FACETS']-1):
                            m.d.sync += facetcnt.eq(0)
                        with m.Else():
                            m.d.sync += [facetcnt.eq(facetcnt+1),
                                         stablecntr.eq(0)]
                        with m.If(var['SINGLE_FACET'] & (facetcnt > 0)):
                            m.next = 'WAIT_END'
                        with m.Else():
                            # TODO: 10 is too high, should be lower
                            thresh = min(round(10.1*ticksinfacet), stableticks)
                            m.d.sync += stablethresh.eq(thresh)
                            m.next = 'READ_INSTRUCTION'
                    with m.Else():
                        m.next = 'WAIT_END'
                with m.Else():
                    m.d.sync += tickcounter.eq(tickcounter+1)
            with m.State('READ_INSTRUCTION'):
                m.d.sync += tickcounter.eq(tickcounter+1)
                with m.If(self.emtpy):
                    m.next = 'WAIT_END'
                with m.Else():
                    m.d.sync += self.read_en.eq(1)
                    m.next = 'PARSE_HEAD'
            with m.State('PARSE_HEAD'):
                m.d.sync += [self.read_en.eq(0), tickcounter.eq(tickcounter+1)]
                with m.If(read_data == INSTRUCTIONS.STOP):
                    m.next = 'STOP'
                with m.Elif(read_data == INSTRUCTIONS.SCAN):
                    m.next = 'WAIT_FOR_DATA_RUN'
                with m.Else():
                    m.d.sync += self.error.eq(1)
                    m.next = 'STOP'
            with m.State('WAIT_FOR_DATA_RUN'):
                m.d.sync += [tickcounter.eq(tickcounter+1),
                             readbit.eq(0),
                             scanbit.eq(0),
                             lasercnt.eq(0)]
                tickcnt_thresh = int(var['START%']*ticksinfacet-2)
                with m.If(self.tickcounter >= tickcnt_thresh):
                    m.next = 'DATA_RUN'
            with m.State('DATA_RUN'):
                m.d.sync += tickcounter.eq(tickcounter+1)
                # NOTE:
                #      readbit is your current position in memory
                #      scanbit current byte position in scanline
                #      lasercnt used to pulse laser at certain freq
                with m.If(lasercnt == 0):
                    with m.If(scanbit >= bitsinscanline):
                        m.next = 'WAIT_END'
                    with m.Else():
                        m.d.sync += [lasercnt.eq(laserticks-1),
                                     scanbit.eq(scanbit+1),
                                     self.lasers[0].eq(read_old[0])]
                # read from memory before the spinup
                # it is triggered here again, so fresh data is available
                # once the end is reached
                # if read bit is 0, trigger a read out unless the next byte
                # is outside of line
                        with m.If(readbit == 0):
                            # TODO: what if fifo is empty!?
                            m.d.sync += [self.read_en.eq(1),
                                         readbit.eq(readbit+1),
                                         read_old.eq(read_old >> 1)]
                        # final read bit copy memory
                        # move to next address, i.e. byte, if end is reached
                        with m.Elif(readbit == MEMWIDTH-1):
                            m.d.sync += read_old.eq(read_data)
                        with m.Else():
                            m.d.sync += [readbit.eq(readbit+1),
                                         read_old.eq(read_old >> 1)]
                with m.Else():
                    m.d.sync += lasercnt.eq(lasercnt-1)
            with m.State('WAIT_END'):
                m.d.sync += [stablecntr.eq(stablecntr+1),
                             tickcounter.eq(tickcounter+1)]
                with m.If(tickcounter >= round(ticksinfacet-jitterticks-1)):
                    m.d.sync += lasers.eq(int('11', 2))
                    m.next = 'STATE_WAIT_STABLE'


class Dispatcher(Elaboratable):
    """ Dispatches instructions to right submodule

        Instructions are buffered in SRAM. This module checks the buffer
        and dispatches the instructions to the corresponding module.
        This is the top module
    """
    def __init__(self, platform=None, divider=50):
        """
            platform  -- used to pass test platform
            divider   -- if sys clk is 50 MHz and divider is 50
                        motor state is update with 1 Mhz
        """
        self.platform = platform
        self.divider = divider

    def elaborate(self, platform):
        m = Module()
        # Connect Parser
        parser = SPIParser(self.platform)
        m.submodules.parser = parser
        # Connect Polynomal Move module
        polynomal = Polynomal(self.platform, self.divider)
        m.submodules.polynomal = polynomal
        # Busy signal
        busy = Signal()
        m.d.comb += busy.eq(polynomal.busy)
        # position adder
        busy_d = Signal()
        m.d.sync += busy_d.eq(polynomal.busy)
        if platform:
            board_spi = platform.request("debug_spi")
            spi = synchronize(m, board_spi)
            m.submodules.car = platform.clock_domain_generator()
            steppers = [res for res in get_all_resources(platform, "stepper")]
            assert len(steppers) != 0
        else:
            platform = self.platform
            self.spi = SPIBus()
            self.parser = parser
            self.pol = polynomal
            spi = synchronize(m, self.spi)
            self.laserhead = platform.laserhead
            self.steppers = steppers = platform.steppers
            self.busy = busy
        coeffcnt = Signal(range(len(polynomal.coeff)))
        # connect laser
        hexa = self.platform.laserhead
        # connect motors
        for idx, stepper in enumerate(steppers):
            m.d.comb += [stepper.step.eq(polynomal.step[idx] &
                                         ((stepper.limit == 0) | stepper.dir)),
                         stepper.dir.eq(polynomal.dir[idx]),
                         parser.pinstate[idx].eq(stepper.limit)]
        # connect spi
        m.d.comb += parser.spi.connect(spi)
        # Polygon
        polygon = Signal()
        # TODO: add timer and move to laserhead
        m.d.comb += hexa.en.eq(polygon)

        with m.If((busy_d == 1) & (busy == 0)):
            for idx, position in enumerate(parser.position):
                m.d.sync += position.eq(position+polynomal.totalsteps[idx])
        with m.FSM(reset='RESET', name='dispatcher'):
            with m.State('RESET'):
                m.next = 'WAIT_INSTRUCTION'
            with m.State('WAIT_INSTRUCTION'):
                m.d.sync += [parser.read_commit.eq(0), polynomal.start.eq(0)]
                with m.If((parser.empty == 0) & parser.execute & (busy == 0)):
                    m.d.sync += parser.read_en.eq(1)
                    m.next = 'PARSEHEAD'
            # check which instruction we r handling
            with m.State('PARSEHEAD'):
                byte0 = parser.read_data[:8]
                with m.If(byte0 == INSTRUCTIONS.MOVE):
                    m.d.sync += [polynomal.ticklimit.eq(parser.read_data[8:]),
                                 parser.read_en.eq(0),
                                 coeffcnt.eq(0)]
                    m.next = 'MOVE_POLYNOMAL'
                with m.Elif(byte0 == INSTRUCTIONS.WRITEPIN):
                    pins = Cat(hexa.laser0, hexa.laser1, polygon)
                    m.d.sync += [pins.eq(parser.read_data[8:]),
                                 parser.read_commit.eq(0)]
                    m.next = 'WAIT_INSTRUCTION'
                with m.Else():
                    m.next = 'ERROR'
                    m.d.sync += parser.dispatcherror.eq(1)
            with m.State('MOVE_POLYNOMAL'):
                with m.If(coeffcnt < len(polynomal.coeff)):
                    with m.If(parser.read_en == 0):
                        m.d.sync += parser.read_en.eq(1)
                    with m.Else():
                        m.d.sync += [polynomal.coeff[coeffcnt].eq(
                                     parser.read_data),
                                     coeffcnt.eq(coeffcnt+1),
                                     parser.read_en.eq(0)]
                with m.Else():
                    m.next = 'WAIT'
                    m.d.sync += [polynomal.start.eq(1),
                                 parser.read_commit.eq(1)]
            # NOTE: you need to wait for busy to be raised
            #       in time
            with m.State('WAIT'):
                m.next = 'WAIT_INSTRUCTION'
            # NOTE: system never recovers user must reset
            with m.State('ERROR'):
                m.next = 'ERROR'
        return m


# Overview:
#  the hardware consists out of the following elements
#  -- SPI command interf
#  -- transactionalized FIFO
#  -- SPI parser (basically an extension of SPI command interf)
#  -- Dispatcher --> dispatches signals to actual hardware
#  -- Polynomal integrator --> determines position via integrating counters

# TODO:
#   -- in practice, position is not reached with small differences like 0.02 mm
#   -- test execution speed to ensure the right PLL is propagated
#   -- luna splits modules over files and adds one test per file
#      this is probably cleaner than put all in one file approach
#   -- use CRC packet for tranmission failure (it is in litex but not luna)
#   -- try to replace value == 0 with ~value
#   -- xfer3 is faster in transaction
#   -- if you chip select is released parsers should return to initial state
#      now you get an error if you abort the transaction
#   -- number of ticks per motor is uniform
#   -- yosys does not give an error if you try to synthesize invalid memory
#   -- read / write commit is not perfect
#   -- simulations do not always agree with reality, around edges
