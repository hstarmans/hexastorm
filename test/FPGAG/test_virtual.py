import unittest
from struct import pack, unpack


from luna.gateware.interface.spi import SPIGatewareTestCase
from luna.gateware.test.utils import sync_test_case

from FPGAG.core import Core
from FPGAG.constants import COMMANDS, MEMDEPTH, BYTESINGCODE, WORD_SIZE
from FPGAG.constants import COMMAND_SIZE

class Test(SPIGatewareTestCase):
    FRAGMENT_UNDER_TEST = Core

    def initialize_signals(self):
        self.dut.fifo = self.dut.spiparser.fifo
        yield self.dut.spi.cs.eq(0)

    def write_command(self, data):
        'convenience function for writing command to controller'
        assert len(data) == (WORD_SIZE+COMMAND_SIZE)/8
        read_data = yield from self.spi_exchange_data(data)
        return unpack('!I', read_data[1:])[0]

    @sync_test_case
    def test_writegcode(self):
        'write directions to the motor pins'
        self.assertEqual((yield self.dut.fifo.empty), 1)
        self.assertEqual((yield self.dut.fifo.space_available), MEMDEPTH)
        # write GCODE command with data
        bytes_sent = 0
        while bytes_sent!=BYTESINGCODE:
            writedata = [COMMANDS.GCODE, 1, 2, 3, 4]
            bytes_sent += 4
            _ = yield from self.spi_exchange_data(writedata)
            self.assertEqual((yield self.dut.fifo.empty), 1)
        while (yield self.dut.fifo.write_commit) == 0:
            yield
        self.assertEqual((yield self.dut.fifo.space_available), MEMDEPTH-BYTESINGCODE/(WORD_SIZE/8))
        self.assertEqual((yield self.dut.fifo.empty), 1)

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
        while (yield self.dut.fifo.write_commit) == 0:
            yield
        yield
        # write execute
        writedata = [COMMANDS.START, COMMANDS.GCODE, 255, 0, 0]
        _ = yield from self.spi_exchange_data(writedata)
        # data should now be parsed and empty become 1
        while (yield self.dut.fifo.empty) == 0:
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
        while (yield self.dut.fifo.write_commit) == 0:
            yield
        yield
        # write execute
        writedata = [COMMANDS.START, COMMANDS.GCODE, 255, 0, 0]
        _ = yield from self.spi_exchange_data(writedata)
        # data should now be parsed and empty become 1
        while (yield self.dut.fifo.empty) == 0:
            yield
        # 2 clocks needed for error to propagate
        yield
        yield
        self.assertEqual((yield self.dut.spiparser.dispatcherror), 1)
        # let's request the status
        bytes_sent = 0
        # TODO: you have jumping rings!!
        while bytes_sent<BYTESINGCODE:
            writedata = [COMMANDS.STATUS, 0, 0, 0, 0]
            bytes_sent += 4
            read_data = yield from self.write_command(writedata)
            print(read_data)


if __name__ == "__main__":
    unittest.main()