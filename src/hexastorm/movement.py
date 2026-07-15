from amaranth import Elaboratable, Module, Mux, Signal, signed
from amaranth.lib.io import Buffer
from amaranth.hdl import Array

from .resources import get_all_resources, StepperRecord


class Polynomial(Elaboratable):
    """Polynomial motor controller

        Evaluates:
            pos(t) = c·t³ + b·t² + a·t

        where t is a counter incremented every clock tick.

        The polynomial describes the stepper position of a single axis.
        A counter is used to capture the state of the polynomial.
        If a given bit, denoted by bitshift, of the counter changes,
        a step is sent to the motor.
        In every tick the step can at most increase
        with one count.

        To achieve high performance and low LUT usage without hardware DSPs,
        this module uses pipelined Discrete Forward Differencing. The host (e.g., ESP32)
        must pre-calculate the forward differences (D1, D2, D3) and submit them
        via the `coeff` array instead of the raw polynomial coefficients.

        The internal accumulators use fixed-point Q-format math. A single large integer
        represents both the fractional and whole steps. The `bit_shift` configuration
        defines the boundary: bits below `bit_shift` are fractional (sub-step resolution),
        and the bit at `bit_shift` triggers a physical motor step when it toggles.
        Non-step fractional parts are preserved across motion segments for perfect continuity.

        The module also supports external laser control. When `override_laser` is high,
        the step and direction signals for the configured laser axis are seamlessly
        hijacked by the external `step_laser` and `dir_laser` inputs.

        Assumptions:
        - Max ticks per move is configured by `move_ticks` (e.g., 10_000).
        - Base update frequency is typically 1 MHz.
        - Absolute position tracking is handled by the host controller.

        Inputs:
        - `coeff`:          Array of coefficients [a, b, c] for each motor.
        - `start`:          Start signal to initiate polynomial evaluation.
        - `tick_limit`:     Maximum number of ticks (duration) for the motion segment.
        - `step_laser`:     External step signal override for the laser axis.
        - `dir_laser`:      External direction signal override for the laser axis.
        - `override_laser`: When high, overrides step/dir for the laser motor.

    Outputs:
        - `busy`:           High while segment evaluation is in progress.
    """

    def __init__(self, plf_cfg):
        """
        plf_cfg  -- platform configuration
        """
        self.plf_cfg = plf_cfg
        hdl_cfg = plf_cfg.hdl_cfg

        # Input
        self.coeff = Array(
            [
                Signal(signed(hdl_cfg.bit_shift + 1))
                for _ in range(hdl_cfg.motors * hdl_cfg.pol_degree)
            ]
        )
        self.start = Signal()
        self.tick_limit = Signal(hdl_cfg.move_ticks.bit_length())
        self.step_laser = Signal()
        self.dir_laser = Signal()
        self.override_laser = Signal()

        # Output
        self.busy = Signal()

        # Mixed
        self.steppers = [StepperRecord() for _ in range(hdl_cfg.motors)]

    def elaborate(self, platform):
        m = Module()
        steppers = self.steppers
        plf_cfg = self.plf_cfg
        hdl_cfg = plf_cfg.hdl_cfg
        ice40_cfg = plf_cfg.ice40_cfg

        # Timing counters
        divider = int(ice40_cfg["clks"][ice40_cfg["hfosc_div"]])
        cntr = Signal(range(divider))
        ticks = Signal(hdl_cfg.move_ticks.bit_length())

        # Internal signed counters per motor per order
        max_steps = int(hdl_cfg.move_ticks / 2)  # Nyquist
        max_bits = (max_steps << hdl_cfg.bit_shift).bit_length()
        cntrs = Array(Signal(signed(max_bits + 1)) for _ in range(len(self.coeff)))
        assert max_bits <= 64

        if platform is None:
            # Expose internals for simulation
            self.ticks = ticks
            self.cntrs = cntrs
        else:
            # Connect to platform stepper resources
            steppers_res = get_all_resources(platform, "stepper", dir="-")
            for i, stepper in enumerate(steppers_res):
                m.submodules += [
                    step_buf := Buffer("o", stepper.step),
                    dir_buf := Buffer("o", stepper.dir),
                    lim_buf := Buffer("i", stepper.limit),
                ]
                m.d.comb += [
                    step_buf.o.eq(steppers[i].step),
                    dir_buf.o.eq(steppers[i].dir),
                    steppers[i].limit.eq(lim_buf.i),
                ]

        # Generate step pulse based on toggling specific bit
        laser_idx = list(plf_cfg.motor_cfg["steps_mm"].keys()).index(
            plf_cfg.motor_cfg["orth2lsrline"]
        )
        for motor in range(hdl_cfg.motors):
            idx = motor * hdl_cfg.pol_degree
            step_motor = cntrs[idx][hdl_cfg.bit_shift]

            with m.If(motor == laser_idx):
                # 1. Pipeline registers for edge detection
                step_motor_d = Signal()
                step_laser_d = Signal()
                m.d.sync += [
                    step_motor_d.eq(step_motor),
                    step_laser_d.eq(self.step_laser),
                ]
                # 2. Detect Rising (Start) and Falling (End) edges for both sources
                # Motor source
                rise_motor = (step_motor == 1) & (step_motor_d == 0)
                fall_motor = (step_motor == 0) & (step_motor_d == 1)
                # Laser source
                rise_laser = (self.step_laser == 1) & (step_laser_d == 0)
                fall_laser = (self.step_laser == 0) & (step_laser_d == 1)

                # 3. Select the *Events* based on the override
                # We switch which edges we listen to, rather than the raw level.
                # This filters out the "jump" glitch when switching sources.
                evt_rise = Signal()
                evt_fall = Signal()
                m.d.comb += [
                    evt_rise.eq(Mux(self.override_laser, rise_laser, rise_motor)),
                    evt_fall.eq(Mux(self.override_laser, fall_laser, fall_motor)),
                ]
                # 4. Reconstruct the signal (SR Latch behavior)
                # This preserves the exact pulse width of the chosen source.
                with m.If(evt_rise):
                    m.d.sync += steppers[motor].step.eq(1)
                with m.Elif(evt_fall):
                    m.d.sync += steppers[motor].step.eq(0)
            with m.Else():
                m.d.sync += steppers[motor].step.eq(step_motor)

        # Direction signal: Purely based on the sign bit of velocity! (0 LUTs)
        for motor in range(hdl_cfg.motors):
            idx = motor * hdl_cfg.pol_degree
            with m.If((motor == laser_idx) & self.override_laser):
                # Override active for laser motor – use external dir
                m.d.sync += steppers[motor].dir.eq(self.dir_laser)
            with m.Else():
                if hdl_cfg.pol_degree >= 2:
                    # Velocity is stored in cntrs[idx + 1]
                    # -1 gets the sign bit. ~ inverts it so positive = 1, negative = 0
                    m.d.sync += steppers[motor].dir.eq(~cntrs[idx + 1][-1])
                else:
                    # Constant velocity is in coeff[idx]
                    m.d.sync += steppers[motor].dir.eq(~self.coeff[idx][-1])

        # State machine for execution
        with m.FSM(init="RESET", name="polynomial_fsm"):
            with m.State("RESET"):
                m.d.sync += self.busy.eq(0)
                m.next = "WAIT_START"

            with m.State("WAIT_START"):
                with m.If(self.start):
                    for motor in range(hdl_cfg.motors):
                        base = motor * hdl_cfg.pol_degree
                        step_bit = hdl_cfg.bit_shift + 1

                        D1 = self.coeff[base]
                        D2 = self.coeff[base + 1] if hdl_cfg.pol_degree > 1 else 0

                        # Load initial Forward Differences based on degree
                        # The host now pre-calculates these, saving massive LUT overhead.
                        if hdl_cfg.pol_degree >= 3:
                            m.d.sync += cntrs[base + 2].eq(D2)
                        if hdl_cfg.pol_degree >= 2:
                            m.d.sync += cntrs[base + 1].eq(D1)

                        # Keep the fractional part from the previous segment for position continuity
                        m.d.sync += cntrs[base].eq(cntrs[base][:step_bit])

                    m.d.sync += self.busy.eq(1)
                    m.next = "RUNNING"
                with m.Else():
                    m.d.sync += self.busy.eq(0)

            with m.State("RUNNING"):
                # Tick handling and polynomial evaluation
                with m.If((ticks < self.tick_limit) & (cntr >= divider - 1)):
                    m.d.sync += [ticks.eq(ticks + 1), cntr.eq(0)]

                    for motor in range(hdl_cfg.motors):
                        base = motor * hdl_cfg.pol_degree

                        # Pipelined Forward Differencing (1 adder per layer per clock tick)

                        if hdl_cfg.pol_degree >= 3:
                            D3 = self.coeff[base + 2]
                            m.d.sync += cntrs[base + 2].eq(cntrs[base + 2] + D3)

                        if hdl_cfg.pol_degree >= 2:
                            D2 = (
                                cntrs[base + 2]
                                if hdl_cfg.pol_degree >= 3
                                else self.coeff[base + 1]
                            )
                            m.d.sync += cntrs[base + 1].eq(cntrs[base + 1] + D2)

                        D1 = (
                            cntrs[base + 1]
                            if hdl_cfg.pol_degree >= 2
                            else self.coeff[base]
                        )
                        m.d.sync += cntrs[base].eq(cntrs[base] + D1)

                with m.Elif(ticks < self.tick_limit):
                    m.d.sync += cntr.eq(cntr + 1)

                with m.Else():
                    m.d.sync += ticks.eq(0)
                    m.next = "WAIT_START"
        return m
