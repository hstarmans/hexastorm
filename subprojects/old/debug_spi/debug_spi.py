from amaranth import Elaboratable, Module
from luna.gateware.interface.spi import SPIDeviceInterface

from hexastorm.platforms import Firestarter
from hexastorm.spi_helpers import connect_synchronized_spi


class DebugSPIExample(Elaboratable):
    """Hardware meant to demonstrate use of
    the Debug Controller's SPI interface copied from Luna"""

    def __init__(self):
        # Base ourselves around an SPI command interface.
        self.interface = SPIDeviceInterface(clock_phase=1)

    def elaborate(self, platform):
        m = Module()
        board_spi = platform.request("debug_spi")

        # Use command interface.
        m.submodules.interface = self.interface

        connect_synchronized_spi(m, board_spi, self.interface)

        # Turn on a single LED, to show something's running.
        led = platform.request("led", 0)
        m.d.comb += led.o.eq(1)

        # Echo back the last received data.
        m.d.comb += self.interface.word_out.eq(self.interface.word_in)

        return m


if __name__ == "__main__":
    platform = Firestarter(micropython=True)
    platform.build(DebugSPIExample(), do_program=False, verbose=True)
