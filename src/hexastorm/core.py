from amaranth import Cat, Elaboratable, Module, Signal, signed, Mux
from amaranth.hdl import Array
from luna.gateware.interface.spi import (
    SPICommandInterface,
)
from luna.gateware.memory import TransactionalizedFIFO


from .config import Spi
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

        self.fifo = TransactionalizedFIFO(
            width=hdl_cfg.mem_width, depth=hdl_cfg.mem_depth
        )

        self.error_dispatch = Signal()
        self.debug_word = Signal(hdl_cfg.mem_width)
        self.parse = Signal()

    def elaborate(self, platform):
        m = Module()
        hdl_cfg = self.hdl_cfg

        m.submodules.interf = spi_cmd = self.spi_command
        m.submodules.fifo = fifo = self.fifo

        if platform is not None:  # Building module
            board_spi = platform.request("debug_spi", dir="-")
            connect_synchronized_spi(m, board_spi, spi_cmd)

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
                with m.If(spi_cmd.command_ready):
                    state_word = Cat(state, self.pin_state)
                    cmd = Spi.Commands
                    with m.Switch(spi_cmd.command):
                        with m.Case(cmd.empty):
                            m.next = "WAIT_COMMAND"
                        with m.Case(cmd.start):
                            m.d.sync += self.parse.eq(1)
                            m.next = "WAIT_COMMAND"
                        with m.Case(cmd.stop):
                            m.d.sync += self.parse.eq(0)
                            m.next = "WAIT_COMMAND"
                        with m.Case(cmd.write):
                            m.d.sync += spi_cmd.word_to_send.eq(state_word)
                            with m.If(state[status.full] == 0):
                                m.next = "WAIT_WORD"
                            with m.Else():
                                m.next = "WAIT_COMMAND"
                        with m.Case(cmd.read):
                            m.d.sync += spi_cmd.word_to_send.eq(state_word)
                            m.next = "WAIT_COMMAND"
                        with m.Case(cmd.debug):
                            m.d.sync += spi_cmd.word_to_send.eq(self.debug_word)
                            m.next = "WAIT_COMMAND"
                        with m.Case(cmd.position):
                            # Position requested multiple times
                            m.d.sync += [
                                spi_cmd.word_to_send.eq(self.position[mtr_idx]),
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
                with m.If(spi_cmd.word_complete):
                    byte0 = spi_cmd.word_received[:8]
                    with m.If(words_rec == 0):
                        valid_instr = (byte0 > 0) & (byte0 < 6)
                        with m.If(valid_instr):
                            m.d.sync += [
                                instr_rec.eq(byte0),
                                fifo.write_en.eq(1),
                                fifo.write_data.eq(spi_cmd.word_received),
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
                            fifo.write_data.eq(spi_cmd.word_received),
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
    """
    Top-level unit that *parses* the instruction FIFO and *dispatches* the work
    either to the laser-head or to the polynomial motion controller.
    """

    def __init__(self, plf_cfg):
        """
        plf_cfg  -- platform configuration
        """
        self.plf_cfg = plf_cfg
        self.busy = Signal()

    def elaborate(self, platform):
        m = Module()

        # shared signals
        enable_prism = Signal()
        lasers = Signal(2)
        read_commit = Signal()
        read_en = Signal()
        read_discard = Signal()
        busy = self.busy  # poly busy or lh processing lines

        # submodules
        m.submodules.parser = parser = SPIParser(self.plf_cfg.hdl_cfg)
        m.submodules.polynomial = poly = Polynomial(self.plf_cfg)

        if platform is None:
            self.parser = parser
            self.pol = poly
            self.lh = m.submodules.laserhead = lh = DiodeSimulator(
                plf_cfg=self.plf_cfg, addfifo=False
            )
        else:
            m.submodules.laserhead = lh = Laserhead(self.plf_cfg)

        # connect laser
        m.d.comb += [
            lh.enable_prism_in.eq(enable_prism),
            lh.lasers_in.eq(lasers),
        ]

        # connect Parser
        m.d.comb += [
            lh.read_data.eq(parser.fifo.read_data),
            lh.empty.eq(parser.fifo.empty),
            parser.fifo.read_commit.eq(read_commit | lh.read_commit),
            parser.fifo.read_en.eq(read_en | lh.read_en),
            parser.fifo.read_discard.eq(read_discard | lh.read_discard),
        ]

        # connect polynomial module
        m.d.comb += [
            poly.step_laser.eq(lh.step),
            poly.dir_laser.eq(lh.dir),
            poly.override_laser.eq(lh.process_lines),
        ]
        for i in range(len(poly.position)):
            m.d.comb += [
                parser.position[i].eq(poly.position[i]),
                parser.pin_state[i].eq(poly.steppers[i].limit),
            ]
        # parser pin state & busy signal
        m.d.comb += [
            parser.pin_state[len(poly.steppers) :].eq(
                Cat(lh.photodiode_t, lh.synchronized)
            ),
            busy.eq(poly.busy | lh.process_lines),
        ]

        # pins you can write to
        self.pins = pins = Cat(lasers, enable_prism, lh.synchronize, lh.singlefacet)
        # poly coeff currently processed
        poly_coeff = Signal(range(len(poly.coeff) + 1))

        with m.FSM(init="RESET", name="dispatcher"):
            with m.State("RESET"):
                m.d.sync += pins.eq(0)
                m.next = "WAIT_INSTRUCTION"

            with m.State("WAIT_INSTRUCTION"):
                m.d.sync += [read_commit.eq(0), poly.start.eq(0)]
                with m.If(~parser.fifo.empty & parser.parse & ~busy):
                    m.d.sync += read_en.eq(1)
                    m.next = "PARSE_HEAD"
            # check which instruction we r handling
            with m.State("PARSE_HEAD"):
                instruction = parser.fifo.read_data[:8]
                payload = parser.fifo.read_data[8:]
                m.d.sync += read_en.eq(0)
                with m.Switch(instruction):
                    instr = Spi.Instructions
                    with m.Case(instr.move):
                        m.d.sync += [
                            poly.tick_limit.eq(payload),
                            poly_coeff.eq(0),
                        ]
                        m.next = "MOVE_POLYNOMIAL"
                    with m.Case(instr.write_pin):
                        m.d.sync += [
                            pins.eq(payload),
                            read_commit.eq(1),
                        ]
                        m.next = "WAIT"
                    with m.Case(instr.scanline, instr.last_scanline):
                        m.d.sync += [
                            read_discard.eq(1),
                            lh.synchronize.eq(1),
                            lh.expose_start.eq(1),
                        ]
                        m.next = "SCANLINE"
                    with m.Default():
                        m.d.sync += parser.error_dispatch.eq(1)
                        m.next = "ERROR"
            with m.State("MOVE_POLYNOMIAL"):
                with m.If(poly_coeff < len(poly.coeff)):
                    with m.If(read_en == 0):
                        m.d.sync += read_en.eq(1)
                    with m.Else():
                        m.d.sync += [
                            poly.coeff[poly_coeff].eq(parser.fifo.read_data),
                            poly_coeff.eq(poly_coeff + 1),
                            read_en.eq(0),
                        ]
                with m.Else():
                    m.d.sync += [poly.start.eq(1), read_commit.eq(1)]
                    m.next = "WAIT"
            with m.State("SCANLINE"):
                m.d.sync += [
                    read_discard.eq(0),
                    lh.expose_start.eq(0),
                ]
                m.next = "WAIT"
            # NOTE: you need to wait for busy to be raised
            #       in time
            with m.State("WAIT"):
                m.d.sync += poly.start.eq(0)
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
