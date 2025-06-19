from types import SimpleNamespace

from amaranth import Cat, Elaboratable, Module, Signal, signed, Mux
from amaranth.hdl import Array
from luna.gateware.interface.spi import (
    SPIBus,
    SPICommandInterface,
)
from luna.gateware.memory import TransactionalizedFIFO

from .config import Spi
from .resources import get_all_resources
from .spi_helpers import connect_synchronized_spi

from .lasers import DiodeSimulator, Laserhead

# from .motor import Driver
from .movement import Polynomial


class SPIParser(Elaboratable):
    """SPI command parser and responder, wrapped by dispatcher

    Supported commands:
        - status   : Return state (parsing, fifo full, error) + pin states
        - start    : Enable parsing (process FIFO)
        - stop     : Disable parsing
        - write    : Write instruction to FIFO
        - read     : Return system state (used with status)
        - debug    : Return debug word
        - position : Cycles through and returns motor positions

    Interface:
        Inputs:
            - spi         : SPIBus interface
            - positions   : Array of stepper motor positions
            - pin_state   : External pin state to report
            - read_commit, read_en, read_discard : FIFO control
            - error_dispatch : Error detected in dispatcher
            - word_to_send  : Debug word to send
        Outputs:
            - parse        : Processing FIFO
            - read_data    : FIFO output
            - empty        : FIFO empty flag
    """

    def __init__(self, hdl_cfg):
        """
        hdl_cfg  -- Hardware defined language configuration (dictionary)
        """
        self.hdl_cfg = hdl_cfg

        self.spi_command = SPICommandInterface(
            command_size=Spi.command_bytes * 8, word_size=Spi.word_bytes * 8
        )
        self.position = Array(Signal(signed(64)) for _ in range(hdl_cfg.motors))
        self.pin_state = Signal(8)

        # FIFO interaction
        self.read_commit = Signal()
        self.read_en = Signal()
        self.read_discard = Signal()

        self.error_dispatch = Signal()
        self.debug_word = Signal(hdl_cfg.mem_width)
        self.parse = Signal()
        self.read_data = Signal(hdl_cfg.mem_width)
        self.empty = Signal()

    def elaborate(self, platform):
        m = Module()
        hdl_cfg = self.hdl_cfg

        interf = self.spi_command
        m.submodules.interf = interf
        self.interf = interf

        # FIFO instantation
        fifo = TransactionalizedFIFO(width=hdl_cfg.mem_width, depth=hdl_cfg.mem_depth)
        m.submodules.fifo = fifo

        if platform is not None:  # Building module
            board_spi = platform.request("debug_spi", dir="-")
            connect_synchronized_spi(m, board_spi, interf)
        else:  # Expose for testing
            self.fifo = fifo

        # Connect FIFO control and status
        m.d.comb += [
            self.read_data.eq(fifo.read_data),
            fifo.read_commit.eq(self.read_commit),
            fifo.read_discard.eq(self.read_discard),
            fifo.read_en.eq(self.read_en),
            self.empty.eq(fifo.empty),
        ]

        # Internal state
        state = Signal(8)
        mtr_idx = Signal(range(hdl_cfg.motors))

        words_rec = Signal(
            range(
                max(
                    hdl_cfg.words_move,
                    hdl_cfg.words_scanline + 1,
                )
            )
        )
        instr_rec = Signal(8)
        word_error = Signal()

        status = Spi.State
        m.d.sync += [
            state[status.parsing].eq(self.parse),
            state[status.full].eq(fifo.space_available <= 1),
            state[status.error].eq(self.error_dispatch | word_error),
        ]

        with m.FSM(name="parser", init="RESET"):
            with m.State("RESET"):
                m.d.sync += [
                    self.parse.eq(1),
                    words_rec.eq(0),
                    word_error.eq(0),
                ]
                m.next = "WAIT_COMMAND"
            with m.State("WAIT_COMMAND"):
                with m.If(interf.command_ready):
                    state_word = Cat(state, self.pin_state)
                    cmd = Spi.Commands
                    with m.Switch(interf.command):
                        with m.Case(cmd.empty):
                            m.next = "WAIT_COMMAND"
                        with m.Case(cmd.start):
                            m.d.sync += self.parse.eq(1)
                            m.next = "WAIT_COMMAND"
                        with m.Case(cmd.stop):
                            m.d.sync += self.parse.eq(0)
                            m.next = "WAIT_COMMAND"
                        with m.Case(cmd.write):
                            m.d.sync += interf.word_to_send.eq(state_word)
                            with m.If(state[status.full] == 0):
                                m.next = "WAIT_WORD"
                            with m.Else():
                                m.next = "WAIT_COMMAND"
                        with m.Case(cmd.read):
                            m.d.sync += interf.word_to_send.eq(state_word)
                            m.next = "WAIT_COMMAND"
                        with m.Case(cmd.debug):
                            m.d.sync += interf.word_to_send.eq(self.debug_word)
                            m.next = "WAIT_COMMAND"
                        with m.Case(cmd.position):
                            # Position requested multiple times
                            m.d.sync += [
                                interf.word_to_send.eq(self.position[mtr_idx]),
                                mtr_idx.eq(
                                    Mux(
                                        mtr_idx < hdl_cfg.motors - 1,
                                        mtr_idx + 1,
                                        0,
                                    )
                                ),
                            ]
                            m.next = "WAIT_COMMAND"
            with m.State("WAIT_WORD"):
                with m.If(interf.word_complete):
                    byte0 = interf.word_received[:8]
                    with m.If(words_rec == 0):
                        valid_instr = (byte0 > 0) & (byte0 < 6)
                        with m.If(valid_instr):
                            m.d.sync += [
                                instr_rec.eq(byte0),
                                fifo.write_en.eq(1),
                                fifo.write_data.eq(interf.word_received),
                                words_rec.eq(words_rec + 1),
                            ]
                            m.next = "WRITE"
                        with m.Else():
                            # Invalid instruction → mark error and discard
                            m.d.sync += word_error.eq(1)
                            m.next = "WAIT_COMMAND"
                    with m.Else():
                        # Additional words for multi-word instructions
                        m.d.sync += [
                            fifo.write_en.eq(1),
                            fifo.write_data.eq(interf.word_received),
                            words_rec.eq(words_rec + 1),
                        ]
                        m.next = "WRITE"
            with m.State("WRITE"):
                m.d.sync += fifo.write_en.eq(0)

                # Define when an instruction is ready to be committed
                ready_to_commit = Signal()

                instr = Spi.Instructions
                m.d.comb += ready_to_commit.eq(
                    ((instr_rec == instr.move) & (words_rec >= hdl_cfg.words_move))
                    | (
                        (instr_rec == instr.scanline)
                        & (words_rec >= hdl_cfg.words_scanline)
                    )
                    | (instr_rec == instr.write_pin)
                    | (instr_rec == instr.last_scanline)
                )

                with m.If(ready_to_commit):
                    m.d.sync += [words_rec.eq(0), fifo.write_commit.eq(1)]
                    m.next = "COMMIT"
                with m.Else():
                    m.next = "WAIT_COMMAND"
            with m.State("COMMIT"):
                m.d.sync += fifo.write_commit.eq(0)
                m.next = "WAIT_COMMAND"
        return m


class Dispatcher(Elaboratable):
    """Dispatches instructions to right submodule

    Instructions are buffered in SRAM. This module checks the buffer
    and dispatches the instructions to the corresponding module.
    This is the top module
    """

    def __init__(self, platform=None):
        """
        platform  -- used to pass test platform
        """
        self.platform = platform
        self.read_commit = Signal()
        self.read_en = Signal()
        self.read_data = Signal(platform.hdl_cfg.mem_width)
        self.read_discard = Signal()
        self.empty = Signal()

    def elaborate(self, platform):
        m = Module()
        platform = self.platform or platform

        parser = m.submodules.parser = SPIParser(platform)
        polynomial = m.submodules.polynomial = Polynomial(platform)

        # Busy used to detect move or scanline in action
        # disabled "dispatching"
        busy = Signal()

        # shared signals
        enable_prism = Signal()
        lasers = Signal(2)

        if platform.settings.test:
            self.spi = spi = SPIBus()
            self.parser = parser
            self.pol = polynomial
            self.lh = lh = platform.laserhead
            self.steppers = steppers = platform.steppers
            self.busy = busy
            lh_mod = m.submodules.laserhead = DiodeSimulator(
                platform=platform, addfifo=False
            )
            self.laserhead = lh_mod
            m.d.comb += [
                lh_mod.enable_prism_in.eq(enable_prism | lh_mod.enable_prism),
                lh_mod.laser0.eq(lasers[0] | lh_mod.lasers[0]),
                lh_mod.laser1.eq(lasers[1] | lh_mod.lasers[1]),
                lh.pwm.eq(lh_mod.pwm),
                lh.en.eq(lh_mod.enable_prism | enable_prism),
                lh.laser0.eq(lh_mod.lasers[0] | lasers[0]),
                lh.laser1.eq(lh_mod.lasers[1] | lasers[1]),
            ]
        else:
            spi = platform.request("debug_spi")
            lh_mod = m.submodules.laserhead = Laserhead(platform)
            lh = platform.request("laserscanner")
            steppers = get_all_resources(platform, "stepper")
            # bldc = platform.request("bldc")
            # leds = [res.o for res in get_all_resources(platform, "led")]
            assert steppers, "No stepper resources found"

            m.d.comb += [
                lh_mod.photodiode.eq(lh.photodiode.i),
                lh.pwm.o.eq(lh_mod.pwm),
                lh.en.o.eq(lh_mod.enable_prism | enable_prism),
                lh.laser0.o.eq(lasers[0] | lh_mod.lasers[0]),
                lh.laser1.o.eq(lasers[1] | lh_mod.lasers[1]),
            ]

        # polynomial iterates over count
        coeffcnt = Signal(range(len(polynomial.coeff) + 1))

        # connect Parser
        m.d.comb += [
            self.read_data.eq(parser.read_data),
            lh_mod.read_data.eq(parser.read_data),
            lh_mod.empty.eq(parser.empty),
            self.empty.eq(parser.empty),
            parser.read_commit.eq(self.read_commit | lh_mod.read_commit),
            parser.read_en.eq(self.read_en | lh_mod.read_en),
            parser.read_discard.eq(self.read_discard | lh_mod.read_discard),
        ]

        normalized_steppers = []
        # Normalize stepper IOs
        for idx, stepper in enumerate(steppers):
            if platform.settings.test:
                norm = SimpleNamespace(
                    step=stepper.step, dir=stepper.dir, limit=stepper.limit
                )
            else:
                norm = SimpleNamespace(
                    step=stepper.step.o, dir=stepper.dir.o, limit=stepper.limit.i
                )
            normalized_steppers.append(norm)

        # Connect logic
        for idx, norm in enumerate(normalized_steppers):
            step = polynomial.step[idx] & ((norm.limit == 0) | norm.dir)

            is_laser_motor = idx == list(
                platform.settings.motor_cfg["steps_mm"].keys()
            ).index(platform.settings.motor_cfg["orth2lsrline"])

            if not is_laser_motor:
                direction = polynomial.dir[idx]
                m.d.comb += [
                    norm.step.eq(step),
                    norm.dir.eq(direction),
                    parser.pin_state[idx].eq(norm.limit),
                ]
            else:
                m.d.comb += [
                    parser.pin_state[idx].eq(norm.limit),
                    norm.step.eq(
                        (step & (~lh_mod.process_lines))
                        | (lh_mod.step & lh_mod.process_lines)
                    ),
                    norm.dir.eq(
                        (polynomial.dir[idx] & (~lh_mod.process_lines))
                        | (lh_mod.dir & lh_mod.process_lines)
                    ),
                ]
        m.d.comb += parser.pin_state[len(steppers) :].eq(
            Cat(lh_mod.photodiode_t, lh_mod.synchronized)
        )

        # update position
        stepper_d = Array(Signal() for _ in range(len(steppers)))
        for idx, stepper in enumerate(normalized_steppers):
            pos = parser.position[idx]
            m.d.sync += stepper_d[idx].eq(stepper.step)
            with m.If(stepper.limit == 1):
                m.d.sync += parser.position[idx].eq(0)
            # assuming position is signed
            # TODO: this might eat LUT, optimize
            pos_max = pow(2, len(pos) - 1) - 2
            with m.Elif((pos > pos_max) | (pos < -pos_max)):
                m.d.sync += parser.position[idx].eq(0)
            with m.Elif((stepper.step == 1) & (stepper_d[idx] == 0)):
                with m.If(stepper.dir):
                    m.d.sync += pos.eq(pos + 1)
                with m.Else():
                    m.d.sync += pos.eq(pos - 1)

        # Busy signal
        m.d.comb += busy.eq(polynomial.busy | lh_mod.process_lines)
        # connect spi
        connect_synchronized_spi(m, spi, parser)

        # pins you can write to
        pins = Cat(lasers, enable_prism, lh_mod.synchronize, lh_mod.singlefacet)
        with m.FSM(init="RESET", name="dispatcher"):
            with m.State("RESET"):
                m.d.sync += pins.eq(0)
                m.next = "WAIT_INSTRUCTION"

            with m.State("WAIT_INSTRUCTION"):
                m.d.sync += [self.read_commit.eq(0), polynomial.start.eq(0)]
                with m.If(
                    (~self.empty)
                    & parser.parse
                    & (~(polynomial.busy | lh_mod.process_lines))
                ):
                    m.d.sync += self.read_en.eq(1)
                    m.next = "PARSEHEAD"
            # check which instruction we r handling
            with m.State("PARSEHEAD"):
                byte0 = self.read_data[:8]
                m.d.sync += self.read_en.eq(0)
                with m.If(byte0 == Spi.Instructions.move):
                    m.d.sync += [
                        polynomial.tick_limit.eq(self.read_data[8:]),
                        coeffcnt.eq(0),
                    ]
                    m.next = "MOVE_POLYNOMIAL"
                with m.Elif(byte0 == Spi.Instructions.write_pin):
                    m.d.sync += [
                        pins.eq(self.read_data[8:]),
                        self.read_commit.eq(1),
                    ]
                    m.next = "WAIT"
                with m.Elif(
                    (byte0 == Spi.Instructions.scanline)
                    | (byte0 == Spi.Instructions.last_scanline)
                ):
                    m.d.sync += [
                        self.read_discard.eq(1),
                        lh_mod.synchronize.eq(1),
                        lh_mod.expose_start.eq(1),
                    ]
                    m.next = "SCANLINE"
                with m.Else():
                    m.next = "ERROR"
                    m.d.sync += parser.error_dispatch.eq(1)
            with m.State("MOVE_POLYNOMIAL"):
                with m.If(coeffcnt < len(polynomial.coeff)):
                    with m.If(self.read_en == 0):
                        m.d.sync += self.read_en.eq(1)
                    with m.Else():
                        m.d.sync += [
                            polynomial.coeff[coeffcnt].eq(self.read_data),
                            coeffcnt.eq(coeffcnt + 1),
                            self.read_en.eq(0),
                        ]
                with m.Else():
                    m.next = "WAIT"
                    m.d.sync += [polynomial.start.eq(1), self.read_commit.eq(1)]
            with m.State("SCANLINE"):
                m.d.sync += [
                    self.read_discard.eq(0),
                    lh_mod.expose_start.eq(0),
                ]
                m.next = "WAIT"
            # NOTE: you need to wait for busy to be raised
            #       in time
            with m.State("WAIT"):
                m.d.sync += polynomial.start.eq(0)
                m.next = "WAIT_INSTRUCTION"
            # NOTE: system never recovers user must reset
            with m.State("ERROR"):
                m.next = "ERROR"
        return m


# Overview:
#  the hardware consists out of the following elements
#  -- SPI command interface
#  -- transactionalized FIFO
#  -- SPI parser (basically an extension of SPI command interface)
#  -- Dispatcher --> dispatches signals to actual hardware
#  -- Polynomial integrator --> determines position via integrating counters

# TODO:
#   -- in practice, position is not reached with small differences like 0.02 mm
#   -- test exucution speed to ensure the right PLL is propagated
#   -- use CRC packet for tranmission failure (it is in litex but not luna)
#   -- try to replace value == 0 with ~value
#   -- xfer3 is faster in transaction
#   -- if you chip select is released parsers should return to initial state
#      now you get an error if you abort the transaction
#   -- number of ticks per motor is uniform
#   -- yosys does not give an error if you try to synthesize invalid memory
#   -- read / write commit is not perfect
#   -- add example of simulating in yosys / chisel
