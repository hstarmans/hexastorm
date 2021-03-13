from nmigen import Signal, Elaboratable, Cat, Module, Const, signed


class Multiplication(Elaboratable):
    def __init__(self):
        self.a = Signal(signed(16))
        self.b = Signal(signed(16))
        self.c = Signal(signed(32))

    def elaborate(self, platform):
        m = Module()
        m.d.sync += [
            self.c.eq(self.a * self.b)
        ]
        return m


class Divisor(Elaboratable):
    """ Euclidean division with a remainder

    X = Y*Q + R
    dividend X by divisor Y you get quotient Q and remainder R
    For a tutorial see https://projectf.io/posts/division-in-verilog/

        width            -- number of bits of divisor and quotients
    I/O signals:
        I: start         -- calculation starts on high
        O: busy          -- calculation in progress
        O: valid         -- result is valid
        O: dbz           -- divide by zero
        I: x             -- dividend
        I: y             -- divisor
        O: q             -- quotients
        O: r             -- remainder
    """
    def __init__(self, width=4):
        self.width = width
        self.start = Signal()
        self.busy = Signal()
        self.valid = Signal()
        self.dbz = Signal()
        self.x = Signal(width)
        self.y = Signal(width)
        self.q = Signal(width)
        self.r = Signal(width)

    def elaborate(self, platform):
        m = Module()
        ac = Signal(self.width+1)
        ac_next = Signal.like(ac)
        temp = Signal.like(ac)
        q1 = Signal(self.width)
        q1_next = Signal.like(q1)
        i = Signal(range(self.width))
        # combinatorial
        with m.If(ac >= self.y):
            m.d.comb += [temp.eq(ac-self.y),
                         Cat(q1_next, ac_next).eq(
                         Cat(1, q1, temp[0:self.width-1]))]
        with m.Else():
            m.d.comb += [Cat(q1_next, ac_next).eq(Cat(q1, ac) << 1)]
        # synchronized
        with m.If(self.start):
            m.d.sync += [self.valid.eq(0), i.eq(0)]
            with m.If(self.y == 0):
                m.d.sync += [self.busy.eq(0),
                             self.dbz.eq(1)]
            with m.Else():
                m.d.sync += [self.busy.eq(1),
                             self.dbz.eq(0),
                             Cat(q1, ac).eq(Cat(Const(0, 1),
                                            self.x, Const(0, self.width)))]
        with m.Elif(self.busy):
            with m.If(i == self.width-1):
                m.d.sync += [self.busy.eq(0),
                             self.valid.eq(1),
                             i.eq(0),
                             self.q.eq(q1_next),
                             self.r.eq(ac_next >> 1)]
            with m.Else():
                m.d.sync += [i.eq(i+1),
                             ac.eq(ac_next),
                             q1.eq(q1_next)]
        return m
