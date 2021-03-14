import unittest
from struct import unpack

from luna.gateware.interface.spi import SPIGatewareTestCase
from luna.gateware.test.utils import sync_test_case
from luna.gateware.test import LunaGatewareTestCase

from FPGAG.controller import Host
from FPGAG.core import Dispatcher, SPIParser, Polynomal
from FPGAG.board import Firestarter, TestPlatform
from FPGAG.constants import (COMMANDS, DEGREE, MOVE_TICKS, BIT_SHIFT,
                             WORD_SIZE, COMMAND_SIZE, INSTRUCTIONS)


class TestPolynomal(LunaGatewareTestCase):
    platform = TestPlatform()
    FRAGMENT_UNDER_TEST = Polynomal
    FRAGMENT_ARGUMENTS = {'platform': platform}

    def count_steps(self, motor):
        '''counts steps in a move with direction'''
        count = 0
        while (yield self.dut.busy):
            old = (yield self.dut.step[motor])
            yield
            if old and ((yield self.dut.step[motor]) == 0):
                if (yield self.dut.dir[motor]):
                    count += 1
                else:
                    count -= 1
        return count

    def steps_compute(self, steps):
        '''computes count for a given number of step

        Shift is needed as two ticks per step is required
        You need to count slightly over the threshold. That is why
        +1 is added.
        '''
        steps = steps << 1
        count = (steps << BIT_SHIFT)+(1 << (BIT_SHIFT-1))
        return count

    def send_coefficients(self, a, b, c):
        '''send coefficients and pulse start

        [a,b,c] for cx^3+bx^2+ax
        '''
        coefs = [a, b, c]
        # load coefficients
        for motor in range(self.platform.motors):
            for coef in range(self.dut.order):
                yield self.dut.coeff[coef].eq(coefs[coef])
        yield from self.pulse(self.dut.start)

    @sync_test_case
    def test_calculation(self, a=2, b=3, c=1):
        ''' Test a simple relation e.g. cx^3+bx^2+ax '''
        yield from self.send_coefficients(a, b, c)
        while (yield self.dut.busy):
            yield
        self.assertEqual((yield self.dut.finished), 1)
        self.assertEqual((yield self.dut.cntrs[0]), a*MOVE_TICKS
                         + b*pow(MOVE_TICKS, 2)+c*pow(MOVE_TICKS, 3))

    @sync_test_case
    def test_jerk(self):
        '''Test lower limit of c, i.e. the jerk

        Smallest value required is defined by pure jerk move
        with 1 step.

        Test if jerk move can be executed with one step.
        '''
        steps = 1
        c = round(self.steps_compute(steps)/pow(MOVE_TICKS, 3))
        yield from self.send_coefficients(0, 0, c)
        while (yield self.dut.busy):
            yield
        self.assertEqual((yield self.dut.finished), 1)
        dut_count = (yield self.dut.cntrs[0])
        self.assertEqual(dut_count >> BIT_SHIFT, steps*2)
        self.assertEqual((yield self.dut.totalsteps[0]), steps)

    @sync_test_case
    def test_move(self):
        '''Movement

        Test forward and backward move with constant speed
        supported in one motion.
        The largest coefficient is determined by pure velocity
        move with half time limit as steps.
        '''
        steps = round(0.4*MOVE_TICKS)
        a = round(self.steps_compute(steps)/MOVE_TICKS)
        yield from self.send_coefficients(a, 0, 0)
        count = (yield from self.count_steps(0))
        self.assertEqual(count, steps)
        dut_count = (yield self.dut.cntrs[0])
        self.assertEqual(dut_count >> BIT_SHIFT, steps*2)
        self.assertEqual((yield self.dut.totalsteps[0]), steps)
        yield from self.send_coefficients(-a, 0, 0)
        count = (yield from self.count_steps(0))
        self.assertEqual(count, -steps)
        dut_count = (yield self.dut.cntrs[0])
        self.assertEqual(dut_count >> BIT_SHIFT, 0)
        self.assertEqual((yield self.dut.totalsteps[0]), 0)


class TestParser(SPIGatewareTestCase):
    platform = TestPlatform()
    FRAGMENT_UNDER_TEST = SPIParser
    FRAGMENT_ARGUMENTS = {'platform': platform}

    def initialize_signals(self):
        self.host = Host(self.platform)
        self.host.spi_exchange_data = self.spi_exchange_data
        yield self.dut.spi.cs.eq(0)

    def write_command(self, data):
        'convenience function for sending command to controller'
        assert len(data) == (WORD_SIZE+COMMAND_SIZE)/8
        read_data = yield from self.spi_exchange_data(data)
        return unpack('!I', read_data[1:])[0]

    @sync_test_case
    def test_writemoveinstruction(self):
        'write move instruction and verify FIFO is no longer empty'
        self.assertEqual((yield self.dut.empty), 1)
        # write move instruction with data
        bytes_sent = 0
        while bytes_sent != self.platform.bytesinmove:
            writedata = [COMMANDS.WRITE, 1, 2, 3, 4]
            bytes_sent += 4
            _ = yield from self.spi_exchange_data(writedata)
        while (yield self.dut.empty) == 1:
            yield
        # Instruction ready
        self.assertEqual((yield self.dut.empty), 0)
        self.assertEqual((yield self.dut.fifo.space_available),
                         (self.platform.memdepth -
                          self.platform.bytesinmove/(WORD_SIZE/8)
                          ))

    @sync_test_case
    def test_memfull(self):
        'write GCODE until memory is full'
        self.assertEqual((yield self.dut.empty), 1)
        # write data
        bytes_sent = 0
        writedata = [COMMANDS.WRITE, 1, 2, 3, 4]
        while bytes_sent < self.platform.bytesinmove*2:
            bytes_sent += 4
            read_data = yield from self.write_command(writedata)
            self.assertEqual(read_data, 0)
        read_data = yield from self.write_command(writedata)
        self.assertEqual(read_data, 1)


class TestDispatcher(SPIGatewareTestCase):
    platform = TestPlatform()
    FRAGMENT_UNDER_TEST = Dispatcher
    FRAGMENT_ARGUMENTS = {'platform': platform}

    def initialize_signals(self):
        yield self.dut.spi.cs.eq(0)

    # NOTE REFACTOR
    def write_instruction(self, data):
        'convenience function for writing command to controller'
        assert len(data) == (WORD_SIZE+COMMAND_SIZE)/8
        read_data = yield from self.spi_exchange_data(data)
        return unpack('!I', read_data[1:])[0]

    @sync_test_case
    def test_invalidinstruction(self):
        '''write invalid instruction and check flag is raised'''
        # write command with invalid data
        bytes_sent = 0
        while bytes_sent != self.platform.bytesinmove:
            writedata = [COMMANDS.WRITE, 0, 0, 0, 0]
            bytes_sent += 4
            yield from self.spi_exchange_data(writedata)
        # wait for data to be committed
        while (yield self.dut.parser.empty) == 1:
            yield
        # enable dispatching of code
        writedata = [COMMANDS.START, 0, 0, 0, 0]
        yield from self.spi_exchange_data(writedata)
        # data should now be prossed from sram and empty become 1
        while (yield self.dut.parser.empty) == 0:
            yield
        # 2 clocks needed for error to propagate
        yield
        yield
        self.assertEqual((yield self.dut.parser.dispatcherror), 1)
        # let's request the status
        bytes_sent = 0
        while bytes_sent != self.platform.bytesinmove:
            writedata = [COMMANDS.STATUS, 0, 0, 0, 0]
            bytes_sent += 4
            read_data = yield from self.write_instruction(writedata)
            # TODO: provide parser for status
            self.assertEqual(read_data, 2)

    @sync_test_case
    def test_instructionreceipt(self):
        'verify command is processed correctly'
        # TODO: you write in the wrong direction!
        writedata = [COMMANDS.WRITE, 0, 0,
                     int('10101010', 2), INSTRUCTIONS.MOVE]
        yield from self.spi_exchange_data(writedata)
        # write coefficients for each motor
        for motor in range(self.platform.motors):
            for coef in range(DEGREE):
                writedata = [COMMANDS.WRITE, 0,
                             0, 0, motor+coef]
                yield from self.spi_exchange_data(writedata)
        # wait till instruction is received
        while (yield self.dut.parser.empty) == 1:
            yield
        yield
        # enable dispatching of buffered code
        writedata = [COMMANDS.START, 0, 0, 0, 0]
        _ = yield from self.spi_exchange_data(writedata)
        # data should now be parsed and empty become 1
        while (yield self.dut.parser.empty) == 0:
            yield
        # confirm receipt
        self.assertEqual((yield self.dut.aux), int('10101010', 2))
        for motor in range(self.platform.motors):
            for coef in range(DEGREE):
                indx = motor*(DEGREE)+coef
                self.assertEqual((yield self.dut.coeff[indx]), motor+coef)


class TestBuild(unittest.TestCase):
    def test_parser(self):
        platform = Firestarter()
        platform.build(SPIParser(), do_program=False, verbose=True)

    def test_dispatcher(self):
        platform = Firestarter()
        platform.build(Dispatcher(), do_program=False, verbose=True)

    def test_polynomal(self):
        platform = Firestarter()
        platform.build(Polynomal(Firestarter()), do_program=False,
                       verbose=True)


if __name__ == "__main__":
    unittest.main()
