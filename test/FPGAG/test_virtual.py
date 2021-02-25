import unittest
from struct import pack, unpack
from math import ceil

from luna.gateware.interface.spi import SPIGatewareTestCase
from luna.gateware.test.utils import sync_test_case

from FPGAG.core import Core, SPIParser
from FPGAG.board import Firestarter, TestPlatform
from FPGAG.resources import StepperRecord
from FPGAG.constants import (COMMANDS,
                             WORD_SIZE, G_CODE, MOTOR_COMMAND,
                             COMMAND_SIZE, WORD_BYTES)

class TestParser(SPIGatewareTestCase):
    platform = TestPlatform()
    FRAGMENT_UNDER_TEST = SPIParser
    FRAGMENT_ARGUMENTS = {'platform': platform}

    def initialize_signals(self):
        yield self.dut.spi.cs.eq(0)

    def write_command(self, data):
        'convenience function for writing command to controller'
        assert len(data) == (WORD_SIZE+COMMAND_SIZE)/8
        read_data = yield from self.spi_exchange_data(data)
        return unpack('!I', read_data[1:])[0]

    @sync_test_case
    def test_writegcode(self):
        'write GCODE and verify fifo is no longer empty'
        self.assertEqual((yield self.dut.empty), 1)
        # write GCODE command with data
        bytes_sent = 0
        while bytes_sent != self.platform.bytesingcode:
            writedata = [COMMANDS.GCODE, 1, 2, 3, 4]
            bytes_sent += 4
            _ = yield from self.spi_exchange_data(writedata)
        while (yield self.dut.empty) == 1:
            yield
        # Instruction ready
        self.assertEqual((yield self.dut.empty), 0)
        self.assertEqual((yield self.dut.fifo.space_available),
                         (self.platform.memdepth
                          -self.platform.bytesingcode/(WORD_SIZE/8)
                          ))

    @sync_test_case
    def test_memfull(self):
        'write GCODE until memory is full'
        self.assertEqual((yield self.dut.empty), 1)
        # write GCODE command with data
        bytes_sent = 0
        writedata = [COMMANDS.GCODE, 1, 2, 3, 4]
        while bytes_sent < self.platform.bytesingcode*2:
            bytes_sent += 4
            read_data = yield from self.write_command(writedata)
            self.assertEqual(read_data, 0)
        read_data = yield from self.write_command(writedata)
        self.assertEqual(read_data, 1)


class TestCore(SPIGatewareTestCase):
    FRAGMENT_UNDER_TEST = Core

    def initialize_signals(self):
        yield self.dut.spi.cs.eq(0)

    # NOTE REFACTOR
    def write_command(self, data):
        'convenience function for writing command to controller'
        assert len(data) == (WORD_SIZE+COMMAND_SIZE)/8
        read_data = yield from self.spi_exchange_data(data)
        return unpack('!I', read_data[1:])[0]

    @sync_test_case
    def test_setdirection(self):
        'write directions to the motor pins'
        self.assertEqual((yield self.dut.directions.dirx), 0)
        self.assertEqual((yield self.dut.directions.diry), 0)
        self.assertEqual((yield self.dut.directions.dirz), 0)
        # write GCODE command with data
        bytes_sent = 0
        while bytes_sent<BYTESINGCODE:
            writedata = [COMMANDS.GCODE, COMMANDS.GCODE, 255, 0, 0]
            bytes_sent += 4
            _ = yield from self.spi_exchange_data(writedata)
        # wait for data to be committed
        while (yield self.dut.parser.empty) == 1:
            yield
        yield
        # write execute
        writedata = [COMMANDS.START, 0, 0, 0, 0]
        _ = yield from self.spi_exchange_data(writedata)
        # data should now be parsed and empty become 1
        while (yield self.dut.parser.empty) == 0:
            yield
        self.assertEqual((yield self.dut.directions.dirx), 1)
        self.assertEqual((yield self.dut.directions.diry), 1)
        self.assertEqual((yield self.dut.directions.dirz), 1)

    @sync_test_case
    def test_invalidcommand(self):
        'write invalid command and test if we can recieve this by querying the status'
        # write invalid GCODE command with data
        bytes_sent = 0
        while bytes_sent<BYTESINGCODE:
            writedata = [COMMANDS.GCODE, 0, 0, 0, 0]
            bytes_sent += 4
            _ = yield from self.spi_exchange_data(writedata)
        # wait for data to be committed
        while (yield self.dut.parser.empty) == 1:
            yield
        # execute code
        writedata = [COMMANDS.START, 0, 0, 0, 0]
        _ = yield from self.spi_exchange_data(writedata)
        # data should now be parsed and empty become 1
        while (yield self.dut.parser.empty) == 0:
            yield
        # 2 clocks needed for error to propagate
        yield
        yield
        self.assertEqual((yield self.dut.parser.dispatcherror), 1)
        # let's request the status
        bytes_sent = 0
        while bytes_sent<BYTESINGCODE:
            writedata = [COMMANDS.STATUS, 0, 0, 0, 0]
            bytes_sent += 4
            read_data = yield from self.write_command(writedata)
            self.assertEqual(read_data, 2)

class TestBuild(unittest.TestCase):
    def test_parser(self):
        platform = Firestarter()
        platform.build(SPIParser(), do_program=False, verbose=True)

    def test_totalbuild(self):
        platform = Firestarter()
        platform.build(Core(), do_program=False, verbose=True)

if __name__ == "__main__":
    unittest.main()
