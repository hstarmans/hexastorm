""" debug spi

Only got it working for older version of luna
git checkout f54de01c9dc13aeca5d20734a9c103227cb9603f
upstream has been notified https://github.com/greatscottgadgets/luna/issues/101
"""
from nmigen import Elaboratable, Module

from FPGAG.platforms import Firestarter

from luna import top_level_cli
from luna.gateware.utils.cdc import synchronize
from luna.gateware.interface.spi import SPIDeviceInterface


class DebugSPIExample(Elaboratable):
    """ Hardware meant to demonstrate use of
        the Debug Controller's SPI interface copied from Luna """

    def __init__(self):

        # Base ourselves around an SPI command interface.
        self.interface = SPIDeviceInterface(clock_phase=1)

    def elaborate(self, platform):
        m = Module()
        board_spi = platform.request("debug_spi")

        # Use command interface.
        m.submodules.interface = self.interface

        # Synchronize and connect SPI.
        spi = synchronize(m, board_spi)
        m.d.comb += self.interface.spi.connect(spi)

        # Turn on a single LED, to show something's running.
        led = platform.request('led', 0)
        m.d.comb += led.eq(1)

        # Echo back the last received data.
        m.d.comb += self.interface.word_out.eq(self.interface.word_in)

        return m


if __name__ == "__main__":
    platform = Firestarter()
    platform.build(DebugSPIExample(), do_program=True, verbose=True)
