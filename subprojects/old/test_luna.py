import random
import struct
import unittest

from luna.gateware.interface.spi import (
    SPICommandInterface,
    SPIGatewareTestCase,
)
from luna.gateware.memory import TransactionalizedFIFO
from luna.gateware.test import LunaGatewareTestCase
from luna.gateware.test.utils import sync_test_case


class TestSPI(SPIGatewareTestCase):
    FRAGMENT_UNDER_TEST = SPICommandInterface
    FRAGMENT_ARGUMENTS = {"command_size": 8, "word_size": 32}

    def initialize_signals(self):
        yield self.dut.spi.cs.eq(0)

    @sync_test_case
    def test_readfrominterface(self):
        "test if you can receive bytes"
        for _ in range(4):
            to_send = random.randint(1, 255)
            yield self.dut.word_to_send.eq(to_send)
            write_data = [1, 255, 0, 0, 1]
            read_data = yield from self.spi_exchange_data(write_data)
            self.assertEqual((yield self.dut.command), 1)
            self.assertEqual(
                (yield self.dut.word_received),
                struct.unpack("!I", bytearray(write_data[1:]))[0],
            )
            self.assertEqual(
                to_send, struct.unpack("!I", bytearray(read_data[1:]))[0]
            )
            print(read_data)


class TestMEM(LunaGatewareTestCase):
    FRAGMENT_UNDER_TEST = TransactionalizedFIFO
    FRAGMENT_ARGUMENTS = {"width": 32, "depth": 8}

    def initialize_signals(self):
        yield self.dut.write_en.eq(0)

    @sync_test_case
    def test_writeandread(self):
        "test if you can receive bytes"
        dut = self.dut
        iterations = 8
        self.assertEqual(
            (yield dut.space_available), self.FRAGMENT_ARGUMENTS["depth"]
        )
        for i in range(iterations):
            yield dut.write_data.eq(i)
            yield from self.pulse(dut.write_en)
        yield from self.pulse(dut.write_commit)
        self.assertEqual(
            (yield dut.space_available),
            self.FRAGMENT_ARGUMENTS["depth"] - iterations,
        )
        for i in range(iterations):
            yield dut.read_en.eq(1)
            yield
            self.assertEqual(i, (yield dut.read_data))
            yield dut.read_en.eq(0)
            yield


if __name__ == "__main__":
    unittest.main()
