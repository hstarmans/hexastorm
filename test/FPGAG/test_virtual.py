import unittest
from random import randint

import numpy as np
from numpy.testing import assert_array_equal
from luna.gateware.interface.spi import SPIGatewareTestCase
from luna.gateware.test.utils import sync_test_case
from luna.gateware.test import LunaGatewareTestCase

from FPGAG.controller import Host
from FPGAG.core import Dispatcher, SPIParser, Polynomal
from FPGAG.board import Firestarter, TestPlatform
from FPGAG.constants import (COMMANDS, DEGREE, MOVE_TICKS, BIT_SHIFT,
                             WORD_BYTES, FREQ)


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
        count = (steps << (1+BIT_SHIFT))+(1 << (BIT_SHIFT-1))
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
        def limittest(limit, steps):
            a = round(self.steps_compute(steps)/limit)
            yield self.dut.ticklimit.eq(limit)
            yield from self.send_coefficients(a, 0, 0)
            while (yield self.dut.busy):
                yield
            self.assertEqual((yield self.dut.totalsteps[0]), steps)
        yield from limittest(5000, 10)
        yield from limittest(2000, 30)

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

        Test forward and backward move at constant speed.
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
    def test_getposition(self):
        decimals = 3
        position = [randint(-2000, 2000) for _ in range(self.platform.motors)]
        for idx, pos in enumerate(self.dut.position):
            yield pos.eq(position[idx])
        lst = (yield from self.host.position).round(decimals)
        stepspermm = np.array(list(self.platform.stepspermm.values()))
        assert_array_equal(lst, (position/stepspermm).round(decimals))

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

    def wait_complete(self):
        '''helper method to wait for completion'''
        cntr = 0
        while (yield self.dut.busy) or (cntr < 100):
            if (yield self.dut.pol.busy):
                cntr = 0
            else:
                cntr += 1
            yield

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
        self.assertEqual((yield from self.host.dispatcherror), True)

    @sync_test_case
    def test_ptpmove(self, steps=[400], ticks=[15_000]):
        'verify point to point move'
        mm = np.array(steps)/np.array(list(self.platform.stepspermm.values()))
        time = np.array(ticks)/FREQ
        speed = mm/time
        yield from self.host.gotopoint(mm.tolist(),
                                       speed.tolist())
        yield from self.wait_complete()
        pos = (yield from self.host.position)
        self.assertEqual(self.host._position, pos)

    @sync_test_case
    def test_movereceipt(self, ticks=10_000, a=1, b=2, c=3):
        'verify move instruction send over with send_move'
        yield from self.host.send_move([ticks], [a], [b], [c])
        # wait till instruction is received
        while (yield self.dut.parser.empty):
            yield
        yield
        # enable dispatching of code
        yield from self.host._executionsetter(True)
        # data should now be parsed and empty become 1
        while (yield self.dut.parser.empty) == 0:
            yield
        # confirm receipt tick limit and coefficients
        self.assertEqual((yield self.dut.pol.ticklimit), 10_000)
        for motor in range(self.platform.motors):
            for coef in range(DEGREE):
                indx = motor*(DEGREE)+coef
                self.assertEqual((yield self.dut.pol.coeff[indx]),
                                 motor+coef+1)
        while (yield self.dut.pol.busy):
            yield
        self.assertEqual((yield self.dut.pol.cntrs[0]), a*ticks
                         + b*pow(ticks, 2)+c*pow(ticks, 3))


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
