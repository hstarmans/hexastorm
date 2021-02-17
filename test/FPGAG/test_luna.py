import unittest
import struct
import random

from luna.gateware.interface.spi import SPIGatewareTestCase, SPICommandInterface
from luna.gateware.test.utils import sync_test_case

class Test(SPIGatewareTestCase):
    FRAGMENT_UNDER_TEST = SPICommandInterface

    def initialize_signals(self):
        yield self.dut.spi.cs.eq(0)

    @sync_test_case
    def test_readfrominterface(self):
        'test if you can receive bytes'
        for _ in range(4):
            to_send = random.randint(1,255)
            yield self.dut.word_to_send.eq(to_send)
            write_data = [1, 255, 0, 0, 1]
            read_data = yield from self.spi_exchange_data(write_data)
            self.assertEqual((yield self.dut.command), 1)
            self.assertEqual((yield self.dut.word_received), struct.unpack('!I', bytearray(write_data[1:]))[0])
            self.assertEqual(to_send, struct.unpack('!I', bytearray(read_data[1:]))[0])
            # problems: I receive \xff??? for command?
if __name__ == "__main__":
    unittest.main()