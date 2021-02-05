""" SRAM synthesis check

If you run the code below an sram is synthesized by Yosys
"""
from nmigen import Elaboratable, Memory, Module, Signal
from FPGAG.board import Firestarter

class Memtest(Elaboratable):
    def __init__(self):
        self.mem = Memory(width=8, depth=512)

    def elaborate(self, platform):
        m = Module()
        led0 = platform.request("led", 0)
        led1 = platform.request("led", 1)
        m.submodules.rdport = rdport = self.mem.read_port()
        with m.If(rdport.data == 255):
            m.d.comb += led0.eq(0)
        with m.Else():
            m.d.comb += led0.eq(1)
        timer = Signal(range(round(100E6)))
        with m.If(timer>100):
            m.d.comb += rdport.addr.eq(100)
        with m.Else():
            m.d.comb += rdport.addr.eq(0)
        m.d.sync += timer.eq(timer+1)
        m.d.comb += led1.o.eq(timer[-1])
        return m


if __name__ == "__main__":
    platform = Firestarter()
    platform.build(Memtest(), do_program=False, verbose=True)
