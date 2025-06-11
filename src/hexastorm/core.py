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

# from .lasers import DiodeSimulator, Laserhead, params
# from .motor import Driver
# from .movement import Polynomial


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
            - dispatcherror : Error detected in dispatcher
            - word_to_send  : Debug word to send
        Outputs:
            - parse        : Processing FIFO
            - read_data    : FIFO output
            - empty        : FIFO empty flag
    """

    def __init__(self, platform, top=False):
        """
        platform  -- pass test platform
        top       -- trigger synthesis of module
        """
        self.platform = platform
        self.top = top
        cfg = platform.hdl_cfg

        self.spi = SPIBus()
        self.position = Array(Signal(signed(64)) for _ in range(cfg.motors))
        self.pin_state = Signal(8)

        # FIFO interaction
        self.read_commit = Signal()
        self.read_en = Signal()
        self.read_discard = Signal()

        self.error_dispatch = Signal()
        self.word_to_send = Signal(cfg.mem_width)
        self.parse = Signal()
        self.read_data = Signal(cfg.mem_width)
        self.empty = Signal()

    def elaborate(self, platform):
        m = Module()

        plf = self.platform or platform
        cfg = plf.hdl_cfg

        if plf and self.top:
            board_spi = plf.request("debug_spi")
            connect_synchronized_spi(m, board_spi, self)

        interf = SPICommandInterface(
            command_size=Spi.command_bytes * 8, word_size=Spi.word_bytes * 8
        )
        m.submodules.interf = interf
        connect_synchronized_spi(m, self.spi, interf)

        # FIFO instantation
        fifo = TransactionalizedFIFO(width=cfg.mem_width, depth=cfg.mem_depth)
        if cfg.test:
            self.fifo = fifo
        m.submodules.fifo = fifo

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
        mtr_idx = Signal(range(cfg.motors))

        words_rec = Signal(
            range(
                max(
                    cfg.words_move,
                    cfg.words_scanline + 1,
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
                    word = Cat(state[::-1], self.pin_state[::-1])
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
                            m.d.sync += interf.word_to_send.eq(word)
                            with m.If(state[status.full] == 0):
                                m.next = "WAIT_WORD"
                            with m.Else():
                                m.next = "WAIT_COMMAND"
                        with m.Case(cmd.read):
                            m.d.sync += interf.word_to_send.eq(word)
                            m.next = "WAIT_COMMAND"
                        with m.Case(cmd.debug):
                            m.d.sync += interf.word_to_send.eq(self.word_to_send)
                            m.next = "WAIT_COMMAND"
                        with m.Case(cmd.position):
                            # Position requested multiple times
                            m.d.sync += [
                                interf.word_to_send.eq(self.position[mtr_idx]),
                                mtr_idx.eq(
                                    Mux(
                                        mtr_idx < cfg.motors - 1,
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
                    ((instr_rec == instr.move) & (words_rec >= cfg.words_move))
                    | (
                        (instr_rec == instr.scanline)
                        & (words_rec >= cfg.words_scanline)
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

    def __init__(self, platform=None, simdiode=False):
        """
        platform  -- used to pass test platform
        """
        self.simdiode = simdiode
        self.platform = platform
        self.read_commit = Signal()
        self.read_en = Signal()
        self.read_data = Signal(platform.hdl_cfg.mem_width)
        self.read_discard = Signal()
        self.empty = Signal()

    def elaborate(self, platform):
        m = Module()
        # Parser
        parser = SPIParser(self.platform)
        m.submodules.parser = parser
        # Busy used to detect move or scanline in action
        # disabled "dispatching"
        busy = Signal()
        # Polynomial Move
        # polynomial = Polynomial(self.platform)
        # m.submodules.polynomial = polynomial
        # if platform:
        #     spi = platform.request("debug_spi")
        #     laserheadpins = platform.request("laserscanner")
        #     steppers = [res for res in get_all_resources(platform, "stepper")]
        #     # bldc = platform.request("bldc")
        #     # leds = [res.o for res in get_all_resources(platform, "led")]
        #     assert len(steppers) != 0
        # else:
        #     platform = self.platform
        #     self.spi = SPIBus()
        #     spi = self.spi
        #     self.parser = parser
        #     self.pol = polynomial
        #     self.laserheadpins = platform.laserhead
        #     self.steppers = steppers = platform.steppers
        #     self.busy = busy
        #     laserheadpins = platform.laserhead
        #     # PCB motor
        #     # bldc = platform.bldc
        #     # leds = platform.leds
        # # Local laser signal clones
        # enable_prism = Signal()
        # lasers = Signal(2)
        # # Laserscan Head
        # if self.simdiode:
        #     laserhead = DiodeSimulator(platform=platform, addfifo=False)
        #     lh = laserhead
        #     m.d.comb += [
        #         lh.enable_prism_in.eq(enable_prism | lh.enable_prism),
        #         lh.laser0in.eq(lasers[0] | lh.lasers[0]),
        #         laserhead.laser1in.eq(lasers[1] | lh.lasers[1]),
        #     ]
        # else:
        #     laserhead = Laserhead(platform=platform)
        #     m.d.comb += laserhead.photodiode.eq(laserheadpins.photodiode.i)
        # m.submodules.laserhead = laserhead
        # if platform.name == "Test":
        #     self.laserhead = laserhead
        # # polynomial iterates over count
        # coeffcnt = Signal(range(len(polynomial.coeff) + 1))
        # ## PCB motor, disabled
        # # # Prism motor
        # # prism_driver = Driver(platform)
        # # m.submodules.prism_driver = prism_driver
        # # # connect prism motor
        # # for idx in range(len(leds)):
        # #     m.d.comb += leds[idx].eq(prism_driver.leds[idx])

        # # m.d.comb += [
        # #     prism_driver.enable_prism.eq(
        # #         enable_prism | laserhead.enable_prism
        # #     ),
        # #     parser.word_to_send.eq(prism_driver.debugword),
        # # ]
        # # m.d.comb += [
        # #     bldc.uL.eq(prism_driver.uL),
        # #     bldc.uH.eq(prism_driver.uH),
        # #     bldc.vL.eq(prism_driver.vL),
        # #     bldc.vH.eq(prism_driver.vH),
        # #     bldc.wL.eq(prism_driver.wL),
        # #     bldc.wH.eq(prism_driver.wH),
        # # ]
        # # m.d.comb += [
        # #     prism_driver.hall[0].eq(bldc.sensor0),
        # #     prism_driver.hall[1].eq(bldc.sensor1),
        # #     prism_driver.hall[2].eq(bldc.sensor2),
        # # ]
        # # # connect laser module to prism motor
        # # m.d.comb += [
        # #     prism_driver.ticksinfacet.eq(laserhead.ticksinfacet),
        # #     prism_driver.synchronized.eq(laserhead.synchronized),
        # # ]

        # # connect laserhead
        # m.d.comb += [
        #     ## Ricoh mirror motor
        #     laserheadpins.pwm.eq(laserhead.pwm),
        #     laserheadpins.en.eq(laserhead.enable_prism | enable_prism),
        #     ## Ricoh mirror motor
        #     laserheadpins.laser0.eq(laserhead.lasers[0] | lasers[0]),
        #     laserheadpins.laser1.eq(laserhead.lasers[1] | lasers[1]),
        # ]
        # # connect Parser
        # m.d.comb += [
        #     self.read_data.eq(parser.read_data),
        #     laserhead.read_data.eq(parser.read_data),
        #     laserhead.empty.eq(parser.empty),
        #     self.empty.eq(parser.empty),
        #     parser.read_commit.eq(self.read_commit | laserhead.read_commit),
        #     parser.read_en.eq(self.read_en | laserhead.read_en),
        #     parser.read_discard.eq(self.read_discard | laserhead.read_discard),
        # ]
        # # connect motors
        # for idx, stepper in enumerate(steppers):
        #     step = polynomial.step[idx] & ((stepper.limit == 0) | stepper.dir)
        #     if idx != (
        #         list(platform.stepspermm.keys()).index(platform.laser_axis)
        #     ):
        #         direction = polynomial.dir[idx]
        #         m.d.comb += [
        #             stepper.step.eq(step),
        #             stepper.dir.eq(direction),
        #             parser.pinstate[idx].eq(stepper.limit),
        #         ]
        #     # connect the motor in which the laserhead moves to laser core
        #     else:
        #         m.d.comb += [
        #             parser.pinstate[idx].eq(stepper.limit),
        #             stepper.step.eq(
        #                 (step & (~laserhead.process_lines))
        #                 | (laserhead.step & (laserhead.process_lines))
        #             ),
        #             stepper.dir.eq(
        #                 (polynomial.dir[idx] & (~laserhead.process_lines))
        #                 | (laserhead.dir & (laserhead.process_lines))
        #             ),
        #         ]
        # m.d.comb += parser.pinstate[len(steppers) :].eq(
        #     Cat(laserhead.photodiode_t, laserhead.synchronized)
        # )

        # # update position
        # stepper_d = Array(Signal() for _ in range(len(steppers)))
        # for idx, stepper in enumerate(steppers):
        #     pos = parser.position[idx]
        #     m.d.sync += stepper_d[idx].eq(stepper.step)
        #     with m.If(stepper.limit == 1):
        #         m.d.sync += parser.position[idx].eq(0)
        #     # assuming position is signed
        #     # TODO: this might eat LUT, optimize
        #     pos_max = pow(2, len(pos) - 1) - 2
        #     with m.Elif((pos > pos_max) | (pos < -pos_max)):
        #         m.d.sync += parser.position[idx].eq(0)
        #     with m.Elif((stepper.step == 1) & (stepper_d[idx] == 0)):
        #         with m.If(stepper.dir):
        #             m.d.sync += pos.eq(pos + 1)
        #         with m.Else():
        #             m.d.sync += pos.eq(pos - 1)

        # # Busy signal
        # m.d.comb += busy.eq(polynomial.busy | laserhead.process_lines)
        # # connect spi
        # connect_synchronized_spi(m, spi, parser)

        # # pins you can write to
        # pins = Cat(lasers, enable_prism, laserhead.synchronize, laserhead.singlefacet)
        # with m.FSM(init="RESET", name="dispatcher"):
        #     with m.State("RESET"):
        #         m.next = "WAIT_INSTRUCTION"
        #         m.d.sync += pins.eq(0)
        #     with m.State("WAIT_INSTRUCTION"):
        #         m.d.sync += [self.read_commit.eq(0), polynomial.start.eq(0)]
        #         with m.If((self.empty == 0) & parser.parse & (busy == 0)):
        #             m.d.sync += self.read_en.eq(1)
        #             m.next = "PARSEHEAD"
        #     # check which instruction we r handling
        #     with m.State("PARSEHEAD"):
        #         byte0 = self.read_data[:8]
        #         m.d.sync += self.read_en.eq(0)
        #         with m.If(byte0 == INSTRUCTIONS.MOVE):
        #             m.d.sync += [
        #                 polynomial.ticklimit.eq(self.read_data[8:]),
        #                 coeffcnt.eq(0),
        #             ]
        #             m.next = "MOVE_POLYNOMIAL"
        #         with m.Elif(byte0 == INSTRUCTIONS.WRITEPIN):
        #             m.d.sync += [
        #                 pins.eq(self.read_data[8:]),
        #                 self.read_commit.eq(1),
        #             ]
        #             m.next = "WAIT"
        #         with m.Elif(
        #             (byte0 == INSTRUCTIONS.SCANLINE)
        #             | (byte0 == INSTRUCTIONS.LASTSCANLINE)
        #         ):
        #             m.d.sync += [
        #                 self.read_discard.eq(1),
        #                 laserhead.synchronize.eq(1),
        #                 laserhead.expose_start.eq(1),
        #             ]
        #             m.next = "SCANLINE"
        #         with m.Else():
        #             m.next = "ERROR"
        #             m.d.sync += parser.dispatcherror.eq(1)
        #     with m.State("MOVE_POLYNOMIAL"):
        #         with m.If(coeffcnt < len(polynomial.coeff)):
        #             with m.If(self.read_en == 0):
        #                 m.d.sync += self.read_en.eq(1)
        #             with m.Else():
        #                 m.d.sync += [
        #                     polynomial.coeff[coeffcnt].eq(self.read_data),
        #                     coeffcnt.eq(coeffcnt + 1),
        #                     self.read_en.eq(0),
        #                 ]
        #         with m.Else():
        #             m.next = "WAIT"
        #             m.d.sync += [polynomial.start.eq(1), self.read_commit.eq(1)]
        #     with m.State("SCANLINE"):
        #         m.d.sync += [
        #             self.read_discard.eq(0),
        #             laserhead.expose_start.eq(0),
        #         ]
        #         m.next = "WAIT"
        #     # NOTE: you need to wait for busy to be raised
        #     #       in time
        #     with m.State("WAIT"):
        #         m.d.sync += polynomial.start.eq(0)
        #         m.next = "WAIT_INSTRUCTION"
        #     # NOTE: system never recovers user must reset
        #     with m.State("ERROR"):
        #         m.next = "ERROR"
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
