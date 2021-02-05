import unittest

from debug_spi import DebugSPIExample

from luna.gateware.interface.spi import SPIGatewareTestCase
from luna.gateware.test.utils import sync_test_case


class SPIDeviceInterfaceTest(SPIGatewareTestCase):
    FRAGMENT_UNDER_TEST = DebugSPIExample

    def initialize_signals(self):
        yield self.dut.spi.cs.eq(0)

    @sync_test_case
    def test_spi_interface(self):
        # do some yields
        for _ in range(10):
            self.assertEqual((yield self.dut.interface.word_complete), 0)
            yield
        # led should be on
        self.assertEqual((yield self.dut.led), 1)
        # write word over spi
        test_bytes = [b"\xAB", b"\xAC"]
        old = b"\x00"
        for test_byte in test_bytes:
            response = yield from self.spi_exchange_data(test_byte)
            self.assertEqual(response, old)
            old = test_byte

if __name__ == "__main__":
    # TODO: doesnt work create a test platform and pass luna that
    unittest.main()