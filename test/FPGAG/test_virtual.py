import unittest

from luna.gateware.interface.spi import SPIGatewareTestCase
from luna.gateware.test.utils import sync_test_case

from FPGAG.core import Core
from FPGAG.constants import COMMANDS, MEMDEPTH

class SPIDeviceInterfaceTest(SPIGatewareTestCase):
    FRAGMENT_UNDER_TEST = Core

    def initialize_signals(self):
        self.dut.spi = self.dut.interface.spi
        yield self.dut.spi.cs.eq(0)

    @sync_test_case
    def test_writegcode(self):
        self.assertEqual((yield self.dut.led), 0)
        # at start buffer is free
        self.assertEqual((yield self.dut.fifo.space_available), MEMDEPTH)
        # write GCODE command with data
        data = [COMMANDS.GCODE, 1, 2, 3]
        response = yield from self.spi_exchange_data(data)
        for _ in range(30):
           yield
        self.assertEqual((yield self.dut.led), 1)
        self.assertEqual((yield self.dut.fifo.space_available), MEMDEPTH-1)


if __name__ == "__main__":
    unittest.main()