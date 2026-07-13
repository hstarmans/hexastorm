from amaranth import Elaboratable, Module, Signal


class HardwarePWM(Elaboratable):
    """
    Hardware PWM generator with an optional clock divider (prescaler)
    to support low-frequency devices like DC fans.
    """

    def __init__(self, resolution=8, clock_div_bits=0):
        self.resolution = resolution
        self.clock_div_bits = clock_div_bits

        # Interface
        self.duty = Signal(resolution)
        self.pwm_out = Signal()

    def elaborate(self, platform):
        m = Module()

        # Base free-running counter for the PWM steps
        counter = Signal(self.resolution)

        if self.clock_div_bits > 0:
            # Create a prescaler counter to slow down the clock
            prescaler = Signal(self.clock_div_bits)
            m.d.sync += prescaler.eq(prescaler + 1)

            # Only tick the main PWM counter when the prescaler overflows (reaches 0)
            with m.If(prescaler == 0):
                m.d.sync += counter.eq(counter + 1)
        else:
            # No divider: tick on every clock cycle (perfect for the spindle)
            m.d.sync += counter.eq(counter + 1)

        # Output logic stays the same
        m.d.comb += self.pwm_out.eq(counter < self.duty)

        return m
