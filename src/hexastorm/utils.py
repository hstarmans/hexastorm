import os
import math
import unittest
from functools import wraps

from amaranth.sim import Simulator


def async_test_case(process_function, *, domain="sync"):
    """Decorator for writing async test cases using Amaranth's new async testbench system."""

    @wraps(process_function)
    def run_test(self):
        self.domain = domain
        self._ensure_clocks_present()

        async def testbench(sim):
            await self.initialize_signals(sim)
            await process_function(self, sim)

        self.sim.add_testbench(testbench)
        self.simulate(vcd_suffix=process_function.__name__)

    return run_test


def usb_domain_test_case(process_function):
    return async_test_case(process_function, domain='usb')


def fast_domain_test_case(process_function):
    return async_test_case(process_function, domain='fast')


def ss_domain_test_case(process_function):
    return async_test_case(process_function, domain='ss')


class LunaGatewareTestCase(unittest.TestCase):

    domain = 'sync'

    FRAGMENT_UNDER_TEST = None
    FRAGMENT_ARGUMENTS = {}

    FAST_CLOCK_FREQUENCY = None
    SYNC_CLOCK_FREQUENCY = 120e6
    USB_CLOCK_FREQUENCY  = None
    SS_CLOCK_FREQUENCY   = None

    def instantiate_dut(self):
        return self.FRAGMENT_UNDER_TEST(**self.FRAGMENT_ARGUMENTS)

    def get_vcd_name(self):
        return f"test_{self.__class__.__name__}"

    def setUp(self):
        self.dut = self.instantiate_dut()
        self.sim = Simulator(self.dut)

        if self.USB_CLOCK_FREQUENCY:
            self.sim.add_clock(1 / self.USB_CLOCK_FREQUENCY, domain="usb")
        if self.SYNC_CLOCK_FREQUENCY:
            self.sim.add_clock(1 / self.SYNC_CLOCK_FREQUENCY, domain="sync")
        if self.FAST_CLOCK_FREQUENCY:
            self.sim.add_clock(1 / self.FAST_CLOCK_FREQUENCY, domain="fast")
        if self.SS_CLOCK_FREQUENCY:
            self.sim.add_clock(1 / self.SS_CLOCK_FREQUENCY, domain="ss")

    async def initialize_signals(self, sim):
        pass  # override in your test class as needed

    def traces_of_interest(self):
        return ()

    def simulate(self, *, vcd_suffix=None):
        if os.getenv('GENERATE_VCDS', default=False):
            vcd_name = self.get_vcd_name()
            if vcd_suffix:
                vcd_name = f"{vcd_name}_{vcd_suffix}"
            traces = self.traces_of_interest()
            with self.sim.write_vcd(vcd_name + ".vcd", vcd_name + ".gtkw", traces=traces):
                self.sim.run()
        else:
            self.sim.run()

    async def pulse(self, sim, signal, *, step_after=True):
        sim.set(signal, 1)
        await sim.tick()
        sim.set(signal, 0)
        if step_after:
            await sim.tick()

    async def advance_cycles(self, sim, cycles):
        for _ in range(cycles):
            await sim.tick()

    async def wait_until(self, sim, strobe, *, timeout=None):
        cycles_passed = 0
        while not sim.get(strobe):
            await sim.tick()
            cycles_passed += 1
            if timeout and cycles_passed > timeout:
                raise RuntimeError(f"Timeout waiting for '{strobe.name}' to go high!")

    def _ensure_clocks_present(self):
        frequencies = {
            'sync': self.SYNC_CLOCK_FREQUENCY,
            'usb':  self.USB_CLOCK_FREQUENCY,
            'fast': self.FAST_CLOCK_FREQUENCY,
            'ss': self.SS_CLOCK_FREQUENCY
        }
        self.assertIsNotNone(frequencies[self.domain], f"no frequency provided for `{self.domain}`-domain clock!")

    async def wait(self, sim, time):
        if self.domain == 'sync':
            period = 1 / self.SYNC_CLOCK_FREQUENCY
        elif self.domain == 'usb':
            period = 1 / self.USB_CLOCK_FREQUENCY
        elif self.domain == 'fast':
            period = 1 / self.FAST_CLOCK_FREQUENCY
        elif self.domain == 'ss':
            period = 1 / self.SS_CLOCK_FREQUENCY
        else:
            raise ValueError(f"Unknown domain: {self.domain}")

        cycles = math.ceil(time / period)
        await self.advance_cycles(sim, cycles)


class LunaUSBGatewareTestCase(LunaGatewareTestCase):
    SYNC_CLOCK_FREQUENCY = None
    USB_CLOCK_FREQUENCY  = 60e6


class LunaSSGatewareTestCase(LunaGatewareTestCase):
    SYNC_CLOCK_FREQUENCY = None
    SS_CLOCK_FREQUENCY   = 125e6
