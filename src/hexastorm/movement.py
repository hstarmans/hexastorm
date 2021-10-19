import unittest

from nmigen import (Signal, Elaboratable, signed, Module)
from nmigen.hdl.mem import Array
from luna.gateware.test import LunaGatewareTestCase, sync_test_case

from hexastorm.resources import get_all_resources
from hexastorm.constants import MOVE_TICKS, bit_shift
from hexastorm.controller import Host
from hexastorm.platforms import TestPlatform


class Polynomal(Elaboratable):
    """ Sets motor states using a polynomal algorithm

        A polynomal up to 3 order, i.e. c*t^3+b*t^2+a*t,
        is evaluated under the assumption that t starts at 0
        and has a maximum of say 10_000 ticks.
        The polynomal describes the stepper position of a single axis.
        A counter is used to capture the state of the polynomal.
        If a given bit, denoted by bitshift, of the counter changes,
        a step is sent to the motor.
        In every tick the step can at most increase
        with one count.

        This code requires a lot of LUT, only order 2 is supported on UP5k
        It is assumed that the user can completely determine
        the outcome of the calculation.
        To ascertain step accuracy, c is submitted with a very high accuracy.
        For third order, this requires 41 bit wide numbers
        and is a "weakness" in the code.
        The code might be sped up via Horner's method and the use of DSPs.
        The current code does not require a DSP.

        Assumptions:
        max ticks per move is 10_000
        update frequency motor is 1 MHz

        I/O signals:
        I: coeff          -- polynomal coefficients
        I: start          -- start signal
        O: busy           -- busy signal
        O: finished       -- finished signal
        O: total steps    -- total steps executed in move
        O: dir            -- direction; 1 is postive and 0 is negative
        O: step           -- step signal
    """
    def __init__(self, platform=None, top=False):
        '''
            platform  -- pass test platform
            top       -- trigger synthesis of module
        '''
        self.top = top
        self.platform = platform
        self.divider = platform.clks[platform.hfosc_div]
        self.order = platform.poldegree
        self.bit_shift = bit_shift(platform)
        self.motors = platform.motors
        self.max_steps = int(MOVE_TICKS/2)  # Nyquist
        # inputs
        self.coeff = Array()
        for _ in range(self.motors):
            self.coeff.extend([Signal(signed(self.bit_shift+1)),
                               Signal(signed(self.bit_shift+1)),
                               Signal(signed(self.bit_shift+1))][:self.order])
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
        # add 1 MHz clock domain
        cntr = Signal(range(self.divider))
        # pos
        max_bits = (self.max_steps << self.bit_shift).bit_length()
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
                         cntrs[motor*self.order][self.bit_shift]),
                         self.totalsteps[motor].eq(
                         cntrs[motor*self.order] >> (self.bit_shift+1))]
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
                        for degree in range(self.order):
                            m.d.sync += cntrs[coef0+degree].eq(0)
                        m.d.sync += counter_d[motor].eq(0)
                    m.d.sync += self.busy.eq(1)
                    m.next = 'RUNNING'
            with m.State('RUNNING'):
                with m.If((ticks < self.ticklimit) & (cntr >= self.divider-1)):
                    m.d.sync += [ticks.eq(ticks+1),
                                 cntr.eq(0)]
                    for motor in range(self.motors):
                        order = self.order
                        idx = motor*order
                        op3, op2, op1 = 0, 0, 0
                        if order > 2:
                            op3 += 3*2*self.coeff[idx+2] + cntrs[idx+2]
                            op2 += cntrs[idx+2]
                            op1 += self.coeff[idx+2] + cntrs[idx+2]
                            m.d.sync += cntrs[idx+2].eq(op3)
                        if order > 1:
                            op2 += (2*self.coeff[idx+1] + cntrs[idx+1])
                            m.d.sync += cntrs[idx+1].eq(op2)
                        op1 += (self.coeff[idx+1]
                                + self.coeff[idx] +
                                cntrs[idx+1] + cntrs[idx])
                        m.d.sync += cntrs[idx].eq(op1)
                with m.Elif(ticks < self.ticklimit):
                    m.d.sync += cntr.eq(cntr+1)
                with m.Else():
                    m.d.sync += ticks.eq(0)
                    m.next = 'WAIT_START'
        return m


class TestPolynomal(LunaGatewareTestCase):
    platform = TestPlatform()
    FRAGMENT_UNDER_TEST = Polynomal
    FRAGMENT_ARGUMENTS = {'platform': platform}

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
        ''' Test wich max speed can be reached

        Suppose max RPM stepper motor is 600, microstepping 16,
        update frequency 1 MHz
         --> (600*60*16)/1E6 = 0.576 step per tick.
        If there are 10_000 ticks per segment;
          5760 steps is maximum you can do in one segment from a physical
          point of view
        At max speed, the motor is on and subsequenctly off
        in subsequent steps, ergo your motor update frequency determines max
        speed (see also Nyquist frequency)
        '''
        def limittest(limit, steps):
            a = round(self.host.steps_to_count(steps)/limit)
            yield self.dut.ticklimit.eq(limit)
            yield from self.send_coefficients(a, 0, 0)
            while (yield self.dut.busy):
                yield
            self.assertEqual((yield self.dut.totalsteps[0]), steps)
        yield from limittest(MOVE_TICKS, 4000)
        yield from limittest(10_000, 1)

    @sync_test_case
    def test_calculation(self, a=2, b=3, c=1):
        ''' Test a simple relation e.g. cx^3+bx^2+ax '''
        if self.dut.order < 3:
            c = 0
        if self.dut.order < 2:
            b = 0
        yield from self.send_coefficients(a, b, c)
        while (yield self.dut.busy):
            yield
        self.assertEqual((yield self.dut.cntrs[0]), a*MOVE_TICKS
                         + b*pow(MOVE_TICKS, 2)+c*pow(MOVE_TICKS, 3))

    @sync_test_case
    def test_jerk(self):
        '''Test lower limit of highest degree, e.g. order 3 the jerk

        Smallest value required is defined by pure higest order move
        with 1 step.
        Test if higest order move can be executed with one step.
        '''
        steps = 1
        coef = round(self.host.steps_to_count(steps)
                     / pow(MOVE_TICKS, self.dut.order))
        coeffs = [0]*3
        coeffs[self.dut.order-1] = coef
        yield from self.send_coefficients(*coeffs)
        while (yield self.dut.busy):
            yield
        dut_count = (yield self.dut.cntrs[0])
        self.assertEqual(dut_count >> self.dut.bit_shift, steps*2)
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
            self.assertEqual(dut_count >> self.dut.bit_shift, steps*2)
            self.assertEqual((yield self.dut.totalsteps[0]), steps)
        steps = round(0.4*MOVE_TICKS)
        yield from do_move(steps)
        yield from do_move(-steps)


if __name__ == "__main__":
    unittest.main()
