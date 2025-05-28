import random
import struct
import unittest

from luna.gateware.interface.spi import (
    SPICommandInterface,
)
from luna.gateware.memory import TransactionalizedFIFO

from hexastorm.spi import SPIGatewareTestCase
from hexastorm.utils import async_test_case, LunaGatewareTestCase


class TestSPI(SPIGatewareTestCase):
    FRAGMENT_UNDER_TEST = SPICommandInterface
    FRAGMENT_ARGUMENTS = {"command_size": 8, "word_size": 32}

    async def initialize_signals(self, sim):
        sim.set(self.dut.spi.cs, 0)
        await sim.tick()

    @async_test_case
    async def test_readfrominterface(self, sim):
        "test if you can receive bytes"
        for _ in range(4):
            to_send = random.randint(1, 255)
            sim.set(self.dut.word_to_send, to_send)
            await sim.tick()
            
            write_data = [1, 255, 0, 0, 1]
            read_data = await self.spi_exchange_data(sim, write_data)
            self.assertEqual(sim.get(self.dut.command), 1)
            self.assertEqual(
                sim.get(self.dut.word_received),
                struct.unpack("!I", bytearray(write_data[1:]))[0],
            )
            self.assertEqual(
                to_send, struct.unpack("!I", bytearray(read_data[1:]))[0]
            )


class TestMEM(LunaGatewareTestCase):
    FRAGMENT_UNDER_TEST = TransactionalizedFIFO
    FRAGMENT_ARGUMENTS = {"width": 32, "depth": 8}

    async def initialize_signals(self, sim):
        sim.set(self.dut.write_en, 0)
        await sim.tick()

    @async_test_case
    async def test_writeandread(self, sim):
        "test if you can receive bytes"
        dut = self.dut
        iterations = 8

        self.assertEqual(
            sim.get(dut.space_available),
            self.FRAGMENT_ARGUMENTS["depth"]
        )

        for i in range(iterations):
            sim.set(dut.write_data, i)
            await self.pulse(sim, dut.write_en)

        await self.pulse(sim, dut.write_commit)

        self.assertEqual(
            sim.get(dut.space_available),
            self.FRAGMENT_ARGUMENTS["depth"] - iterations
        )

        for i in range(iterations):
            self.assertEqual(sim.get(dut.read_data), i)
            sim.set(dut.read_en, 1)
            await sim.tick()
            sim.set(dut.read_en, 0)
            await sim.tick()

if __name__ == "__main__":
    unittest.main()
