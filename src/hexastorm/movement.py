import unittest

from nmigen import (Signal, Elaboratable, signed, Module)
from nmigen.hdl.mem import Array
from luna.gateware.test import LunaGatewareTestCase, sync_test_case

from hexastorm.resources import get_all_resources
from hexastorm.constants import (DEGREE, BIT_SHIFT,
                                 MOVE_TICKS)
from hexastorm.controller import Host
from hexastorm.platforms import TestPlatform


class Polynomal(Elaboratable):
    """ Sets motor states using a polynomal algorithm

        A polynomal up to 3 order, e.g. c*x^3+b*x^2+a*x,
        is evaluated using the assumption that x starts at 0
        and y starts at 0. The polynomal determines the stepper
        position. The bitshift bit determines
        the position. In every tick the step can at most increase
        with one count.

        I/O signals:
        I: coeff          -- polynomal coefficients
        I: start          -- start signal
        O: busy           -- busy signal
        O: finished       -- finished signal
        O: total steps    -- total steps executed in move
        O: dir            -- direction; 1 is postive and 0 is negative
        O: step           -- step signal
    """
    def __init__(self, platform=None, divider=50, top=False):
        '''
            platform  -- pass test platform
            divider -- original clock of 100 MHz via PLL reduced to 50 MHz
                       if this is divided by 50 motor state updated
                       with 1 Mhz
            top       -- trigger synthesis of module
        '''
        self.top = top
        self.divider = divider
        self.platform = platform
        self.order = DEGREE
        # change code for other orders
        assert self.order == 3
        self.motors = platform.motors
        self.max_steps = int(MOVE_TICKS/2)  # Nyquist
        # inputs
        self.coeff = Array()
        for _ in range(self.motors):
            self.coeff.extend([Signal(signed(64)),
                               Signal(signed(64)),
                               Signal(signed(64))])
        self.start = Signal()
        self.ticklimit = Signal(MOVE_TICKS.bit_length())
        # output
        self.busy = Signal()
        self.totalsteps = Array(Signal(signed(self.max_steps.bit_length()+1))
                                for _ in range(self.motors))
        self.dir = Array(Signal() for _ in range(self.motors))
        self.step = Array(Signal() for _ in range(self.motors))

    def elaborate(self, platform):
        m = Module()
        # add 1 MHZ clock domain
        cntr = Signal(range(self.divider))
        # pos
        max_bits = (self.max_steps << BIT_SHIFT).bit_length()
        cntrs = Array(Signal(signed(max_bits+1))
                      for _ in range(len(self.coeff)))
        assert max_bits <= 64
        ticks = Signal(MOVE_TICKS.bit_length())
        if self.top:
            steppers = [res for res in get_all_resources(platform, "stepper")]
            assert len(steppers) != 0
            for idx, stepper in enumerate(steppers):
                m.d.comb += [stepper.step.eq(self.step[idx]),
                             stepper.dir.eq(self.dir[idx])]
        else:
            self.ticks = ticks
            self.cntrs = cntrs

        # steps
        for motor in range(self.motors):
            m.d.comb += [self.step[motor].eq(
                         cntrs[motor*self.order][BIT_SHIFT]),
                         self.totalsteps[motor].eq(
                         cntrs[motor*self.order] >> (BIT_SHIFT+1))]
        # directions
        counter_d = Array(Signal(signed(max_bits+1))
                          for _ in range(self.motors))
        for motor in range(self.motors):
            m.d.sync += counter_d[motor].eq(cntrs[motor*self.order])
            # negative case --> decreasing
            with m.If(counter_d[motor] > cntrs[motor*self.order]):
                m.d.sync += self.dir[motor].eq(0)
            # positive case --> increasing
            with m.Elif(counter_d[motor] < cntrs[motor*self.order]):
                m.d.sync += self.dir[motor].eq(1)
        with m.FSM(reset='RESET', name='polynomen'):
            with m.State('RESET'):
                m.next = 'WAIT_START'
                m.d.sync += self.busy.eq(0)
            with m.State('WAIT_START'):
                m.d.sync += self.busy.eq(0)
                with m.If(self.start):
                    for motor in range(self.motors):
                        coef0 = motor*self.order
                        m.d.sync += [cntrs[coef0+2].eq(0),
                                     cntrs[coef0+1].eq(0),
                                     cntrs[coef0].eq(0),
                                     counter_d[motor].eq(0)]
                    m.d.sync += self.busy.eq(1)
                    m.next = 'RUNNING'
            with m.State('RUNNING'):
                with m.If((ticks < self.ticklimit) & (cntr >= self.divider-1)):
                    m.d.sync += [ticks.eq(ticks+1),
                                 cntr.eq(0)]
                    for motor in range(self.motors):
                        idx = motor*self.order
                        op3 = 3*2*self.coeff[idx+2] + cntrs[idx+2]
                        op2 = (cntrs[idx+2] + 2*self.coeff[idx+1]
                               + cntrs[idx+1])
                        op1 = (self.coeff[idx+2] + self.coeff[idx+1]
                               + self.coeff[idx] + cntrs[idx+2] +
                               cntrs[idx+1] + cntrs[idx])
                        m.d.sync += [cntrs[idx+2].eq(op3),
                                     cntrs[idx+1].eq(op2),
                                     cntrs[idx].eq(op1)]
                with m.Elif(ticks < self.ticklimit):
                    m.d.sync += cntr.eq(cntr+1)
                with m.Else():
                    m.d.sync += ticks.eq(0)
                    m.next = 'WAIT_START'
        return m


class TestPolynomal(LunaGatewareTestCase):
    platform = TestPlatform()
    FRAGMENT_UNDER_TEST = Polynomal
    FRAGMENT_ARGUMENTS = {'platform': platform, 'divider': 1}

    def initialize_signals(self):
        self.host = Host(self.platform)
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
            a = round(self.host.steps_to_count(steps)/limit)
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
        c = round(self.host.steps_to_count(steps)/pow(MOVE_TICKS, 3))
        yield from self.send_coefficients(0, 0, c)
        while (yield self.dut.busy):
            yield
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
            a = round(self.host.steps_to_count(steps)/MOVE_TICKS)
            yield from self.send_coefficients(a, 0, 0)
            count = (yield from self.count_steps(0))
            self.assertEqual(count, steps)
            dut_count = (yield self.dut.cntrs[0])
            self.assertEqual(dut_count >> BIT_SHIFT, steps*2)
            self.assertEqual((yield self.dut.totalsteps[0]), steps)
        steps = round(0.4*MOVE_TICKS)
        yield from do_move(steps)
        yield from do_move(-steps)


if __name__ == "__main__":
    unittest.main()
