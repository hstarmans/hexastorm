import unittest
from time import ticks_ms
from math import isclose
from asyncio import sleep, run

from ulab import numpy as np

from hexastorm.config import Spi
from hexastorm.fpga_host.interface import Memfull
from hexastorm.fpga_host.micropython import ESP32Host
from hexastorm.ulabext import assert_array_almost_equal


def async_test(fn):
    """Decorator: run an async test method inside the event loop."""

    def wrapper(*args, **kwargs):
        return run(fn(*args, **kwargs))

    return wrapper


class Base(unittest.TestCase):
    """Fixture: create one FPGA host for all derived tests."""

    @classmethod
    def setUpClass(cls):
        cls.host = ESP32Host()
        run(cls.host.reset())

    @classmethod
    def tearDownClass(cls):
        run(cls.host.reset())


class StaticTest(Base):
    @async_test
    async def test_memfull(self):
        """Fill FIFO to the brim and make sure it empties again."""
        host = self.host
        hdl_cfg = host.cfg.hdl_cfg
        await host.set_parsing(False)
        host.enable_steppers = False
        host.spi_tries = 10
        for _ in range(hdl_cfg.mem_depth):
            coeff = [3] * hdl_cfg.motors
            try:
                await host.spline_move(hdl_cfg.move_ticks, coeff)
            except Memfull:
                pass
        host.spi_tries = 100_000
        self.assertTrue((await host.fpga_state)["mem_full"])
        await host.set_parsing(True)
        fpga_state = await host.fpga_state
        self.assertFalse(fpga_state["mem_full"])
        self.assertFalse(fpga_state["error"])

    @async_test
    async def test_invalid_instruction(self):
        """Send invalid instrucctions; expect error flag but no mem_full"""
        host = self.host
        command = [Spi.Commands.write] + [0] * Spi.word_bytes
        for _ in range(host.cfg.hdl_cfg.words_move):
            await host.send_command(command)
        await sleep(3)
        fpga_state = await host.fpga_state
        self.assertFalse(fpga_state["mem_full"])
        self.assertTrue(fpga_state["error"])


class LaserheadTest(Base):
    def test_setlaserpower(self, power=130):
        """Write & read laser-driver digipot via I²C (device 0x28)."""
        self.host.laser_current = power
        assert self.host.laser_current == power

    @async_test
    async def spinprism(self, time_sec=15):
        "Spin prism for time_sec seconds."
        host = self.host
        await host.enable_comp(polygon=True)
        print(f"Spinning prism for {time_sec} seconds")
        await sleep(time_sec)
        await host.enable_comp(polygon=False)
        self.assertFalse((await host.fpga_state)["error"])

    @async_test
    async def lasertest(self, laser1=True):
        """Manually enable/disable laser, watching error flag.

        Args:
            lasers1: use laser channel 1 instead of 0
        """
        host = self.host
        print(f"Press enter to turn laser {1 if laser1 else 0} on")
        input()
        await host.enable_comp(laser1=laser1, laser0=(not laser1))
        print("Press enter to turn laser off")
        input()
        await host.enable_comp(laser1=False, laser0=False)
        self.assertFalse((await host.fpga_state)["error"])

    @async_test
    async def test_diode(self, time_out=3):
        """Verify photodiode fires when laser & polygon run."""
        host = self.host
        self.assertFalse((await host.fpga_state)["photodiode_trigger"])
        await host.enable_comp(laser1=True, polygon=True)
        print(f"Wait for diode trigger, {time_out} seconds")
        await sleep(time_out)
        await host.enable_comp(laser1=False, polygon=False)
        fpga_state = await host.fpga_state
        self.assertTrue(fpga_state["photodiode_trigger"])
        self.assertFalse(fpga_state["error"])

    @async_test
    async def test_stable(self, timeout=3):
        """Use eyesight to verify the laserhead is properly locked."""
        host = self.host
        await host.enable_comp(synchronize=True)
        print(f"Wait for synchronization, {timeout} seconds")
        await sleep(timeout)
        self.assertTrue((await host.fpga_state)["photodiode_trigger"])
        await host.enable_comp(synchronize=False)
        self.assertFalse((await host.fpga_state)["error"])

    @async_test
    async def test_move(self, dist=10, steps_line=1, time_out=3):
        """Move ±*dist* mm while scanning and verify end-position.

        Args:
            dist:       displacement in mm
            steps_line: steps per line
            timeout:    wait
        """
        host = self.host
        laz_tim = host.cfg.laser_timing
        mt_cfg = host.cfg.motor_cfg
        numb_lines = round(
            dist * mt_cfg["steps_mm"][mt_cfg["orth2lsrline"]] * steps_line
        )
        await host.enable_comp(synchronize=True)
        start_pos = (await host.position).copy()
        host.enable_steppers = True
        print(f"Wait for synchronization, {time_out} seconds.")
        await sleep(time_out)
        line = [0] * laz_tim["scanline_length"]
        for direction in [0, 1]:
            for _ in range(numb_lines):
                await host.write_line(line, steps_line, direction)
            print(f"Wait for move to complete, {time_out} seconds.")
            sleep(time_out)
            indx = list(mt_cfg["steps_mm"].keys())
            indx = indx.index(mt_cfg["orth2lsrline"])
            # assume y axis is 1
            start_pos[indx] += dist if direction else -dist
            assert_array_almost_equal((await host.position), start_pos, decimal=1)
        await host.write_line([])
        print(f"Wait for stopline to execute, {time_out} seconds")
        await sleep(time_out)
        self.assertFalse((await host.fpga_state)["error"])
        await host.enable_comp(synchronize=False)
        self.host.enable_steppers = False
        # Note: parsing is left open
        # (yield from self.host.set_parsing(False))

    @async_test
    async def test_scanline(self, numb_lines=1_000, repeat=True, singlefacet=False):
        """Send *numb_lines* scanlines and compare measured line rate."""
        host = self.host
        laz_tim = host.cfg.laser_timing
        line = [1] * laz_tim["scanline_length"]
        await host.enable_comp(singlefacet=singlefacet)
        # Pre-load buffer a bit
        for _ in range(8):
            await host.write_line(line)
        print(f"Processing {numb_lines} lines")
        await sleep(3)
        start_ms = ticks_ms()
        if repeat:
            await host.write_line(line, repetitions=numb_lines)
        else:
            for _ in range(numb_lines):
                await host.write_line(line)
        elapsed_s = (ticks_ms() - start_ms) / 1000
        measured_hz = numb_lines / elapsed_s
        await host.write_line([])
        # 3000 RPM
        #    100 khz --> pass
        #    400 Khz --> pass, fail for repeat is False
        expected_freq = laz_tim["rpm"] / 60
        if not singlefacet:
            expected_freq *= laz_tim["facets"]
        print(
            f"line rate measured: {measured_hz:.2f},  expected {expected_freq:.2f} in Hertz"
        )
        # measured freq is higher as it still need to clean the buffer
        self.assertTrue(isclose(measured_hz, expected_freq, rel_tol=0.1))
        self.assertFalse((await host.fpga_state)["error"])
        await host.enable_comp(synchronize=False)


class MoveTest(Base):
    """Core movement and limit-switch sanity checks."""

    @async_test
    async def read_pin(self):
        """Continuously print limit-switch status until Ctrl-C.

        Place a sheet of paper in the cavity of the optical switch and verify the trigger.
        """
        self.host.enable_steppers = False
        try:
            while True:
                st = await self.host.fpga_state
                print(f"[x, y, z] = [{st['x']}, {st['y']}, {st['z']}]")
                await sleep(1)
        except KeyboardInterrupt:
            pass

    def motor_enable(self):
        """Toggle `enable_steppers` and ask the user to feel the brake."""
        self.host.enable_steppers = True
        print("Axes should be locked now — press <Enter> when confirmed.")
        input()
        self.host.enable_steppers = False

    @async_test
    async def test_multiple_move(self, decimals=1):
        """Jog ±10 mm twice and verify final position is unchanged."""
        motors = self.host.platform.motors
        delta = np.array([10, 10, 0])
        start_pos = (await self.host.position).copy()
        # does not work with -1, 1
        for direction in [1, -1]:
            self.assertFalse((await self.host.fpga_state)["error"])
            self.host.enable_steppers = True
            start_move_pos = (await self.host.position).copy()
            disp = delta * direction
            await self.host.gotopoint(position=disp, speed=[1] * motors, absolute=False)
            await sleep(3)
            assert_array_almost_equal(
                await self.host.position,
                start_move_pos + disp,
                decimal=decimals,
            )
        assert_array_almost_equal(
            (await self.host.position), start_pos, decimal=decimals
        )
        self.host.enable_steppers = False
