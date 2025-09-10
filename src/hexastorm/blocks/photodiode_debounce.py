from amaranth import Elaboratable, Module, Signal
from amaranth.lib.io import Buffer


class PhotodiodeDebounce(Elaboratable):
    """
    Debounce + one-shot for an active-low photodiode.
    Photodiode signal is 2FF-synchronized to the FPGA clock.
    This is not done during testing (platform=None).

    Params:
      n_low  : consecutive lows required to fire
      n_high : consecutive highs required to re-arm
    """

    def __init__(self, n_low=10, n_high=4):
        self.n_low = int(n_low)
        self.n_high = int(n_high)

        # Inputs
        self.raw = Signal()  # asynchronous / noisy (active-high == no light)

        # Outputs
        self.sync_level = Signal()  # 2FF-synchronized level of raw
        self.valid_pulse = Signal()  # one-cycle pulse when low stays for n_low
        self.in_refractory = Signal()  # high until we see n_high highs
        self.low_streak = Signal(range(self.n_low + 1))
        self.high_streak = Signal(range(self.n_high + 1))

    def elaborate(self, platform):
        m = Module()

        meta = Signal()
        if platform is not None:
            # Ensure the photodiode pin is set as input
            pd = platform.request("photodiode", dir="-")
            m.submodules += [
                phd_buf := Buffer("i", pd.photodiode),
            ]
            m.d.comb += self.raw.eq(phd_buf.i)
            # 2FF synchronizer
            # probability that metastability (photodiode hit at edge window)
            # input stuck in a inbetween
            # state lasts across two full
            # clock periods is astronomically low.
            m.d.sync += [
                meta.eq(self.raw),
                self.sync_level.eq(meta),
            ]
        else:
            # 2FF synchronizer disabled for testing
            m.d.comb += self.sync_level.eq(self.raw)

        # streak counters (saturating)
        with m.If(~self.sync_level):  # active-low light detected
            with m.If(self.low_streak < self.n_low):
                m.d.sync += self.low_streak.eq(self.low_streak + 1)
            m.d.sync += self.high_streak.eq(0)
        with m.Else():
            with m.If(self.high_streak < self.n_high):
                m.d.sync += self.high_streak.eq(self.high_streak + 1)
            m.d.sync += self.low_streak.eq(0)

        # defaults
        m.d.sync += self.valid_pulse.eq(0)

        # fire once when we *reach* n_low and not refractory
        with m.If((self.low_streak == self.n_low) & ~self.in_refractory):
            m.d.sync += [
                self.valid_pulse.eq(1),
                self.in_refractory.eq(1),
            ]

        # re-arm after n_high highs
        with m.If(self.in_refractory & (self.high_streak == self.n_high)):
            m.d.sync += self.in_refractory.eq(0)

        return m
