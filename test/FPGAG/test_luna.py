import unittest
import struct

from luna.gateware.interface.spi import SPIGatewareTestCase, SPICommandInterface
from luna.gateware.test.utils import sync_test_case

class Test(SPIGatewareTestCase):
    FRAGMENT_UNDER_TEST = SPICommandInterface

    def initialize_signals(self):
        yield self.dut.spi.cs.eq(0)

    @sync_test_case
    def test_readfrominterface(self):
        'test if you can receive bytes'
        yield self.dut.word_to_send.eq(16)
        for _ in range(4):
            write_data = [1, 255, 0, 0, 1]
            read_data = yield from self.spi_exchange_data(write_data)
            self.assertEqual((yield self.dut.command), 1)
            self.assertEqual((yield self.dut.word_received), struct.unpack('!I', bytearray(write_data[1:]))[0])
            # problems: 1. word to send to does not come back
            #           2. cs magically gets low during transaction 
            print(read_data)
            # This is printed
            #    bytearray(b'\x00\x00\x00\x00\x10')
            #    bytearray(b'\xff\x00\x00\x00\x10')
            #    bytearray(b'\xff\x00\x00\x00\x10')
            #    bytearray(b'\xff\x00\x00\x00\x10')
            # problems: I receive \xff??? and \x10 instead of 16
if __name__ == "__main__":
    unittest.main()