""" Sketch of Bezier algorithm on FPGA
"""
import os

from amaranth import Array, Elaboratable, Module, Signal, signed

from hexastorm.platforms import Firestarter


class Casteljau(Elaboratable):
    """Implements Bezier curves using Casteljau's algorithm

    Once calculation is done a signal is raised.
    User can obtain result by looking at beta zero.
    """

    def __init__(self, order=3, totalbits=16, fractionalbits=5):
        # Fixed point arithmic
        # https://vha3.github.io/FixedPoint/FixedPoint.html
        self.totalbits = totalbits
        self.fractionalbits = fractionalbits
        self.signed = 1
        # Bernstein coefficients
        self.coeff = Array()
        for _ in order:
            self.coeff.extend([Signal(signed(totalbits))])
        # time
        self.t = Signal(signed(totalbits))
        # In / out signals
        self.done = Signal()
        self.beta = Array().like(self.coeff)

    def elaborate(self, platform):
        m = Module()

        beta = self.beta
        temp = Signal(signed(self.totalbits * 2))

        n = len(self.coeff)
        j = Signal(range(1, n))
        k = Signal.like(j)
        with m.FSM(reset="INIT"):
            with m.State("INIT"):
                m.d.sync += self.done.eq(0)
                for i in range(n):
                    m.d.sync += beta[i].eq(self.coeff[i])
                m.d.sync += [k.eq(0), j.eq(1)]
                m.next = "UPDATE"
            with m.FSM("UPDATE"):
                m.d.sync += temp.eq(
                    beta[k] * (1 - self.t) + beta[k + 1] * self.t
                )
                m.next = "MULTIPLICATIONFIX"
            # Fixed point arithmetic need fix
            # see multiplication as https://vha3.github.io/FixedPoint/FixedPoint.html
            with m.FSM("MULTIPLICATIONFIX"):
                m.d.sync += beta[k].eq(
                    temp[
                        self.fractionalbits : self.fractionalbits
                        + self.totalbits
                    ]
                )
                with m.If(k != n - j):
                    m.d.sync += k.eq(k + 1)
                    m.next = "UPDATE"
                with m.Else():
                    with m.If(j != n):
                        m.d.sync += j.eq(j + 1)
                        m.d.sync += k.eq(0)
                        m.next = "UPDATE"
                    with m.Else():
                        m.next = "FINISH"
            with m.FSM("FINISH"):
                m.d.sync += self.done.eq(1)
                m.next = "FINISH"
        return m


def de_casteljau(t, coefs):
    """Berstein polynomal as defined by the Casteljau algorithm

    see https://en.wikipedia.org/wiki/De_Casteljau%27s_algorithm
    """
    beta = [c for c in coefs]  # values in this list are overridden
    n = len(beta)
    for j in range(1, n):
        for k in range(n - j):
            beta[k] = beta[k] * (1 - t) + beta[k + 1] * t
    return beta[0]


if __name__ == "__main__":
    # Force yosys to use DSP slices.
    os.environ["NMIGEN_synth_opts"] = "-dsp"
    platform = Firestarter()
    # needs to be less than 30 Mhz
    # BEGINT MET INFO
    # LLC 113, DSP 1, pass op 24 MHZ
    platform.hfosc_div = 2
    # print(help(platform.build))
    # look at *.tim
    # platform.build(Multest(), do_program=True, verbose=False)
