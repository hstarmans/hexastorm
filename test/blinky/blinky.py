from nmigen import Module, Elaboratable, Signal
from board import Firestarter

class Blinky(Elaboratable):
    def elaborate(self, platform):
        led = platform.request("led")
        timer = Signal(range(round(100E6)))

        m = Module()
        m.d.sync += timer.eq(timer+1)
        m.d.comb += led.o.eq(timer[-1])
        return m

if __name__ == '__main__':
    platform = Firestarter()
    platform.build(Blinky(), do_program=False, verbose=False)
