import unittest
from struct import pack, unpack
from math import ceil

from luna.gateware.interface.spi import SPIGatewareTestCase
from luna.gateware.test.utils import sync_test_case
from luna.gateware.test import LunaGatewareTestCase

from FPGAG.core import Dispatcher, SPIParser, Polynomal
from FPGAG.board import Firestarter, TestPlatform
from FPGAG.resources import StepperRecord
from FPGAG.constants import (COMMANDS, BEZIER_DEGREE,
                             WORD_SIZE, G_CODE,
                             COMMAND_SIZE, WORD_BYTES)


class TestPolynomal(LunaGatewareTestCase):
    platform = TestPlatform()
    FRAGMENT_UNDER_TEST = Polynomal
    FRAGMENT_ARGUMENTS = {'platform': platform,
                          'max_time': 10_000,
                          'motors': platform.motors}

    @sync_test_case
    def test_calculation(self, a=2, b=3, c=4):
        ''' Test a simple relation e.g. cx^3+bx^2+ax '''
        coefs = [a, b, c]
        numb_coeff = self.platform.motors*self.dut.order
        # load coefficients
        for motor in range(self.platform.motors):
            for coef in range(self.dut.order):
                yield self.dut.coeff[coef].eq(coefs[coef])
        yield from self.pulse(self.dut.start)
        while (yield self.dut.busy) == 1:
            yield
        max_time = self.FRAGMENT_ARGUMENTS['max_time']
        self.assertEqual((yield self.dut.finished), 1)
        self.assertEqual((yield self.dut.counters[0]), a*max_time+b*pow(max_time, 2)+c*pow(max_time, 3))


    @sync_test_case
    def test_accuracy(self):
        '''Movement

        Assuming, coefficients are constant along a move 
        the position is defined with;
            x = v*t + 1/2*a*t^2 + 1/3*1/2*b*t^3
        Here, v is velocity, a is acceleration and b is jerk.
        You sent to the controller however;
            x = a*t + b*t^2 + c*t^3
        Assume there can be no more than 10_000 ticks in a move,
        Max accuracy required is defined by c . For a pure c move, one step
        c would be 1E-12. The bitshift is set set at 40+1 bits.
        Hence, coefficients can not be smaller than 1E-12!
        Step speed must be lower than 1/2 oscillator speed (Nyquist criterion)
        So for a typical stepper motor (https://blog.prusaprinters.org/calculator_3416/)
        with 400 steps per mm, max speed is 3.125 m/s with an
        oscillator frequency of 1 MHz.
        If other properties are desired, alter max_ticks per step, bit_length or
        oscillator frequency.

        # Suppose you want to do one tick
        1>>43
        '''
        c = round((1<<41)/pow(10_000, 3))
        coefs = [0, 0, c]
        numb_coeff = self.platform.motors*self.dut.order
        # load coefficients
        for motor in range(self.platform.motors):
            for coef in range(self.dut.order):
                yield self.dut.coeff[coef].eq(coefs[coef])
        yield from self.pulse(self.dut.start)
        while (yield self.dut.busy) == 1:
            yield
        max_time = self.FRAGMENT_ARGUMENTS['max_time']
        self.assertEqual((yield self.dut.finished), 1)
        count = (yield self.dut.counters[0])
        self.assertEqual(int(bin(count)[-41]), 1)


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


class TestDispatcher(SPIGatewareTestCase):
    platform = TestPlatform()
    FRAGMENT_UNDER_TEST = Dispatcher
    FRAGMENT_ARGUMENTS = {'platform': platform}

    def initialize_signals(self):
        yield self.dut.spi.cs.eq(0)

    # NOTE REFACTOR
    def write_command(self, data):
        'convenience function for writing command to controller'
        assert len(data) == (WORD_SIZE+COMMAND_SIZE)/8
        read_data = yield from self.spi_exchange_data(data)
        return unpack('!I', read_data[1:])[0]
   
    @sync_test_case
    def test_invalidcommand(self):
        '''verify invalid spi command via spi'''
        # write invalid GCODE command with data
        bytes_sent = 0
        while bytes_sent != self.platform.bytesingcode:
            writedata = [COMMANDS.GCODE, 0, 0, 0, 0]
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
        while bytes_sent != self.platform.bytesingcode:
            writedata = [COMMANDS.STATUS, 0, 0, 0, 0]
            bytes_sent += 4
            read_data = yield from self.write_command(writedata)
            # TODO: provide parser for status
            self.assertEqual(read_data, 2)
      
    @sync_test_case
    def test_commandreceival(self):
        'verify command is processed correctly'
        #TODO: you write in the wrong direction!
        writedata = [COMMANDS.GCODE, COMMANDS.GCODE,
                     int('10101010', 2), 0, 0]
        yield from self.spi_exchange_data(writedata)
        # write coefficients for each motor
        for motor in range(self.platform.motors):
            for coef in range(BEZIER_DEGREE+1):
                writedata = [COMMANDS.GCODE, 0,
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
        # confirm receival
        self.assertEqual((yield self.dut.aux), int('10101010', 2))
        for motor in range(self.platform.motors):
            for coef in range(BEZIER_DEGREE+1):
                indx = motor*(BEZIER_DEGREE+1)+coef
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
        platform.build(Polynomal(Firestarter()), do_program=False, verbose=True)

if __name__ == "__main__":
    unittest.main()
