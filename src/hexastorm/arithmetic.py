# code was built as exercise, it is used nowhere in the project
# should probably be removed
from amaranth import Cat, Const, Elaboratable, Module, Signal, signed


class Multiplication(Elaboratable):
    """Signed 16-bit multiplier producing a 32-bit result."""

    def __init__(self):
        self.a = Signal(signed(16))
        self.b = Signal(signed(16))
        self.c = Signal(signed(32))

    def elaborate(self, platform):
        m = Module()
        m.d.sync += self.c.eq(self.a * self.b)
        return m


class Divisor(Elaboratable):
    """Unsigned Euclidean division (restoring algorithm).

    Performs: X = Y * Q + R

    For a tutorial see https://projectf.io/posts/division-in-verilog/

    Attributes:
        width (int): bit-width of operands

    Inputs:
        start (1):       trigger to start division
        x (width):       dividend
        y (width):       divisor

    Outputs:
        q (width):       quotient
        r (width):       remainder
        busy (1):        division in progress
        valid (1):       result is ready
        dbz (1):         divide-by-zero flag
    """

    def __init__(self, width=4):
        self.width = width

        # Inputs
        self.start = Signal()
        self.x = Signal(width)
        self.y = Signal(width)

        # Outputs
        self.q = Signal(width)
        self.r = Signal(width)
        self.busy = Signal()
        self.valid = Signal()
        self.dbz = Signal()

    def elaborate(self, platform):
        m = Module()

        total_width = self.width + 1

        # Internal signals
        acc = Signal(total_width)  # Accumulator
        acc_next = Signal.like(acc)
        temp = Signal.like(acc)
        q_reg = Signal(self.width)  # Partial quotient
        q_next = Signal.like(q_reg)
        i = Signal(range(self.width + 1))  # Iteration counter

        # --- Combinatorial logic ---
        with m.If(acc >= self.y):
            m.d.comb += [
                temp.eq(acc - self.y),
                Cat(q_next, acc_next).eq(
                    Cat(Const(1, 1), q_reg, temp[: self.width - 1])
                ),
            ]
        with m.Else():
            m.d.comb += [Cat(q_next, acc_next).eq(Cat(q_reg, acc) << 1)]

        # --- Sequential logic ---
        with m.If(self.start):
            m.d.sync += [
                self.valid.eq(0),
                i.eq(0),
            ]
            with m.If(self.y == 0):
                m.d.sync += [
                    self.busy.eq(0),
                    self.dbz.eq(1),
                ]
            with m.Else():
                m.d.sync += [
                    self.busy.eq(1),
                    self.dbz.eq(0),
                    Cat(q_reg, acc).eq(Cat(Const(0, 1), self.x, Const(0, self.width))),
                ]

        with m.Elif(self.busy):
            with m.If(i == self.width - 1):
                m.d.sync += [
                    self.q.eq(q_next),
                    self.r.eq(acc_next >> 1),
                    self.busy.eq(0),
                    self.valid.eq(1),
                    i.eq(0),
                ]
            with m.Else():
                m.d.sync += [
                    i.eq(i + 1),
                    acc.eq(acc_next),
                    q_reg.eq(q_next),
                ]

        return m
