# the led is on --> can you find this
import unittest

from nmigen import Signal, Module, Cat, Elaboratable, Record
from nmigen.hdl.ast import Rose, Fell
from nmigen.hdl.rec import DIR_FANIN, DIR_FANOUT
from debug_spi import DebugSPIExample

from luna.gateware.interface.spi import SPIGatewareTestCase
from luna.gateware.test.utils import LunaGatewareTestCase, sync_test_case

class SPIBus(Record):
    """ Record representing an SPI bus. """

    def __init__(self):
        super().__init__([
            ('sck', 1, DIR_FANIN),
            ('sdi', 1, DIR_FANIN),
            ('sdo', 1, DIR_FANOUT),
            ('cs',  1, DIR_FANIN)
        ])


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
    unittest.main()