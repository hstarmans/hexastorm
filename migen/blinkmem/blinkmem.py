"""
    blinkmem.py
    The memory is initialized with 1 or 0.
    The FPGA reads from the memory.
    If the memory is 1 the led is on and off otherwise.
    
    Rik starmans
"""
from migen.fhdl import *
from migen import *
import sys
sys.path.append("..") 
import icezero as board


class BlinkMem(Module):
    def __init__(self, led):
        self.led = led
        self.specials.mem = Memory(1, 8, 8*[1])
        val = Signal()
        p1 = self.mem.get_port(write_capable=True)
        p2 = self.mem.get_port(has_re=True)
        self.specials += p1, p2
        self.ios = {p1.adr, p1.dat_w, p1.we, p2.dat_r, p2.adr, p2.re}
        self.submodules.transmission = FSM (reset_state = "IDLE")
        self.finish = Signal()
        self.transmission.act("IDLE",
            NextValue(p2.adr,0),
            NextValue(p2.re, 1),
            NextState("READ"))
        self.transmission.act("READ",
            NextValue(val,p2.dat_r),
            NextValue(p2.re, 0),
            NextState("CONDITION"))
        self.transmission.act("CONDITION",
            NextValue(self.finish, 1),
            If((val==1),
            NextValue(self.led, 1)
              ).
            Else(NextValue(self.led, 0)),
            NextState("IDLE"))


def test_bench(mod):
    while (yield mod.finish) == 0:
        yield
    yield
    assert (yield mod.led) == 0


if __name__ == "__main__":
    plat = board.Platform()
    led = plat.request("user_led")
    m = BlinkMem(led)
    import sys
    if len(sys.argv)>1:
        if sys.argv[1] == 'sim':
            run_simulation(m, test_bench(m))
    else:
        plat.build(m, run=True)
