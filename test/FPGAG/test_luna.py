import unittest
import struct
import random


from FPGAG.constants import GCODE, MOTOR_COMMAND

from luna.gateware.interface.spi import SPIGatewareTestCase, SPICommandInterface

from luna.gateware.test.utils import sync_test_case




class Test(SPIGatewareTestCase):
    FRAGMENT_UNDER_TEST = SPICommandInterface
    FRAGMENT_ARGUMENTS = {'command_size':8, 'word_size':32}
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
            print(read_data)
            # problems: I receive \xff??? for command? this problems dissappears if sdo is set.
            # bytearray(b'\x00\x00\x00\x00\x1f')
            # bytearray(b'\xff\x00\x00\x00W')
            # bytearray(b'\xff\x00\x00\x00/')
            # bytearray(b'\xff\x00\x00\x00\xde')
if __name__ == "__main__":
    unittest.main()