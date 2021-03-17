import unittest

from luna.gateware.interface.spi import SPIGatewareTestCase
from luna.gateware.test.utils import sync_test_case
from luna.gateware.test import LunaGatewareTestCase

from FPGAG.controller import Host
from FPGAG.core import Dispatcher, SPIParser, Polynomal
from FPGAG.board import Firestarter, TestPlatform
from FPGAG.constants import (COMMANDS, DEGREE, MOVE_TICKS, BIT_SHIFT,
                             WORD_BYTES)


class TestPolynomal(LunaGatewareTestCase):
    platform = TestPlatform()
    FRAGMENT_UNDER_TEST = Polynomal
    FRAGMENT_ARGUMENTS = {'platform': platform}

    def initialize_signals(self):
        yield self.dut.ticklimit.eq(MOVE_TICKS)

    def count_steps(self, motor):
        '''counts steps in accounting for direction'''
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
        '''compute count for a given number of steps

        steps  -- motor moves in small steps

        Shift is needed as two ticks per step are required
        You need to count slightly over the threshold. That is why
        +1 is added.
        '''
        steps = steps << 1
        count = (steps << BIT_SHIFT)+(1 << (BIT_SHIFT-1))
        return count

    def send_coefficients(self, a, b, c):
        '''send coefficients and pulse start

        a,b,c --  for cx^3+bx^2+ax
        '''
        coefs = [a, b, c]
        # load coefficients
        for motor in range(self.platform.motors):
            for coef in range(self.dut.order):
                yield self.dut.coeff[coef].eq(coefs[coef])
        yield from self.pulse(self.dut.start)

    @sync_test_case
    def test_ticklimit(self):
        ''' Test different upper tick limits'''
        def limittest(limit):
            yield self.dut.ticklimit.eq(limit)
            yield from self.send_coefficients(1, 0, 0)
            while (yield self.dut.busy):
                yield
            self.assertEqual((yield self.dut.cntrs[0]), limit)
        yield from limittest(10)
        yield from limittest(30)

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

        Test forward and backward move with constant speed.
        The largest constant in polynomal is determined by pure
        velocity move with half time limit as steps.
        '''
        def do_move(steps):
            # NOTE: (a = s/t) != -1*(-s/t)
            #       might be due to rounding and bitshift
            a = round(self.steps_compute(steps)/MOVE_TICKS)
            yield from self.send_coefficients(a, 0, 0)
            count = (yield from self.count_steps(0))
            self.assertEqual(count, steps)
            dut_count = (yield self.dut.cntrs[0])
            self.assertEqual(dut_count >> BIT_SHIFT, steps*2)
            self.assertEqual((yield self.dut.totalsteps[0]), steps)
        steps = round(0.4*MOVE_TICKS)
        yield from do_move(steps)
        yield from do_move(-steps)


class TestParser(SPIGatewareTestCase):
    platform = TestPlatform()
    FRAGMENT_UNDER_TEST = SPIParser
    FRAGMENT_ARGUMENTS = {'platform': platform}

    def initialize_signals(self):
        self.host = Host(self.platform)
        self.host.spi_exchange_data = self.spi_exchange_data
        yield self.dut.spi.cs.eq(0)

    @sync_test_case
    def test_writemoveinstruction(self):
        'write move instruction and verify FIFO is no longer empty'
        self.assertEqual((yield self.dut.empty), 1)
        yield from self.host.send_move([1000], [1], [2], [3])
        while (yield self.dut.empty) == 1:
            yield
        # Instruction ready
        self.assertEqual((yield self.dut.empty), 0)
        self.assertEqual((yield self.dut.fifo.space_available),
                         (self.platform.memdepth -
                          self.platform.bytesinmove/WORD_BYTES
                          ))

    @sync_test_case
    def test_readpinstate(self):
        '''retrieve pin state'''
        dct = (yield from self.host.pinstate)
        dct['x'] = 1
        dct['z'] = 1
        bitlist = dct.values()
        b = int("".join(str(i) for i in bitlist), 2)
        yield self.dut.pinstate.eq(b)
        yield
        newdct = (yield from self.host.pinstate)
        self.assertDictEqual(dct, newdct)

    @sync_test_case
    def test_enableparser(self):
        '''enables SRAM parser via command and verifies status with
        different command'''
        yield from self.host._executionsetter(True)
        self.assertEqual((yield self.dut.execute), 1)
        self.assertEqual((yield from self.host.execution), True)

    @sync_test_case
    def test_memfull(self):
        'write move instruction until memory is full'
        self.assertEqual((yield self.dut.empty), 1)
        # platform memory is on default 2 move instructions
        yield from self.host.send_move([1000], [1], [2], [3])
        yield from self.host.send_move([1000], [1], [2], [3])
        writedata = [COMMANDS.WRITE]+[1]*WORD_BYTES
        read_data = yield from self.host.send_command(writedata)
        self.assertEqual(read_data, 1)


class TestDispatcher(SPIGatewareTestCase):
    platform = TestPlatform()
    FRAGMENT_UNDER_TEST = Dispatcher
    FRAGMENT_ARGUMENTS = {'platform': platform}

    def initialize_signals(self):
        self.host = Host(self.platform)
        self.host.spi_exchange_data = self.spi_exchange_data
        yield self.dut.spi.cs.eq(0)

    @sync_test_case
    def test_invalidwrite(self):
        '''write invalid instruction and verify error is raised'''
        command = [COMMANDS.WRITE] + [0]*WORD_BYTES
        for _ in range(DEGREE+1):
            yield from self.host.send_command(command)
        # enable dispatching of code
        yield from self.host._executionsetter(True)
        # data should now be processed from sram and empty become 1
        while (yield self.dut.parser.empty) == 0:
            yield
        # 2 clocks needed for error to propagate
        yield
        yield
        self.assertEqual((yield self.dut.parser.dispatcherror), True)

    @sync_test_case
    def test_moveinstructionreceipt(self):
        'verify move instruction is parsed correctly'
        yield from self.host.send_move([0], [1], [2], [3])
        # wait till instruction is received
        while (yield self.dut.parser.empty) == 1:
            yield
        yield
        # enable dispatching of code
        yield from self.host._executionsetter(True)
        # data should now be parsed and empty become 1
        while (yield self.dut.parser.empty) == 0:
            yield
        # confirm receipt
        for motor in range(self.platform.motors):
            for coef in range(DEGREE):
                indx = motor*(DEGREE)+coef
                self.assertEqual((yield self.dut.coeff[indx]), motor+coef+1)


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
