from migen.fhdl import verilog
from migen import *


class Memtest(Module):
    def __init__(self, led):
        self.specials.mem = Memory(8, 512, init = [255])
        self.specials.p1 = self.mem.get_port(write_capable=True, mode = READ_FIRST)
        self.comb += If(self.p1.dat_r == 255,
                        led.eq(0)
                    ).Else(led.eq(1))

led = Signal()
memtest = Memtest(led)
print(verilog.convert(memtest, ios={led}))