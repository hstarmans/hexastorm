import unittest

from luna.gateware.interface.spi import SPIGatewareTestCase
from luna.gateware.test.utils import sync_test_case

from FPGAG.core import Core
from FPGAG.constants import COMMANDS, MEMDEPTH, BYTESINGCODE, WORD_SIZE

class SPIDeviceInterfaceTest(SPIGatewareTestCase):
    FRAGMENT_UNDER_TEST = Core

    def initialize_signals(self):
        self.dut.spi = self.dut.interface.spi
        yield self.dut.spi.cs.eq(0)

    @sync_test_case
    def test_writegcode(self):
        self.assertEqual((yield self.dut.fifo.empty), 1)
        self.assertEqual((yield self.dut.fifo.space_available), MEMDEPTH)
        # write GCODE command with data
        bytes_sent = 0
        if bytes_sent<BYTESINGCODE:
            writedata = [COMMANDS.GCODE, 1, 2, 3, 4]
            bytes_sent += 4
            _ = yield from self.spi_exchange_data(writedata)
            self.assertEqual((yield self.dut.fifo.empty), 1)
        # wait for data to be committed
        if (yield self.dut.fifo.write_commit) == 0:
            yield
        yield
        self.assertEqual((yield self.dut.fifo.space_available), MEMDEPTH-(BYTESINGCODE/WORD_SIZE))
        self.assertEqual((yield self.dut.fifo.empty), 0)

if __name__ == "__main__":
    unittest.main()