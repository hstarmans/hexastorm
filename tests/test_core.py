from random import randint

import numpy as np
from numpy.testing import assert_array_almost_equal, assert_array_equal

from hexastorm.config import Spi, PlatformConfig
from hexastorm.utils import async_test_case
from hexastorm.spi import SPIGatewareTestCase
from hexastorm.controller import TestHost, Memfull
from hexastorm.core import SPIParser, Dispatcher


class TestParser(SPIGatewareTestCase):
    hdl_cfg = PlatformConfig(test=True).hdl_cfg
    FRAGMENT_UNDER_TEST = SPIParser
    FRAGMENT_ARGUMENTS = {"hdl_cfg": hdl_cfg}

    async def initialize_signals(self, sim):
        self.host = TestHost()
        self.sim = sim
        self.dut.spi = self.dut.spi_command.spi
        self.host.spi_exchange_data = self.spi_exchange_data
        sim.set(self.dut.spi.cs, 0)
        await sim.tick()

    async def assert_fifo_written(self, check):
        """
        Waits for the FIFO to become non-empty and verifies that the expected
        number of bytes have been written.

        Args:
            expected_bytes (int): Number of bytes expected to be in the FIFO.
        """
        sim = self.sim
        await self.wait_until(~self.dut.fifo.empty)
        self.assertEqual(
            sim.get(self.dut.fifo.space_available),
            self.hdl_cfg.mem_depth - check,
        )

    @async_test_case
    async def test_position_readout(self, sim):
        """
        Checks that host-reported motor positions match values set on FPGA accounting for mm conversion.
        """
        decimals = 3
        position = [randint(-2000, 2000) for _ in range(self.hdl_cfg.motors)]
        for idx, pos in enumerate(self.dut.position):
            sim.set(pos, position[idx])
        await sim.tick()
        lst = (await self.host.position).round(decimals)
        steps_mm = np.array(list(self.host.cfg.motor_cfg["steps_mm"].values()))
        assert_array_equal(lst, (np.array(position) / steps_mm).round(decimals))

    @async_test_case
    async def test_scanline_write_to_fifo(self, sim):
        """
        Verifies that writing a full scanline sends the expected number of words to the FIFO.
        """
        laser_timing = self.host.cfg.laser_timing
        await self.host.write_line([1] * laser_timing["scanline_length"])
        await self.wait_until(~self.dut.fifo.empty)
        await self.assert_fifo_written(self.hdl_cfg.words_scanline)

    @async_test_case
    async def test_scanline_empty_to_fifo(self, sim):
        """
        Verifies that an empty scanline triggers a stop command (1 word written to FIFO).
        """
        await self.host.write_line([])
        await self.assert_fifo_written(1)

    @async_test_case
    async def test_enable_comp_triggers_write(self, sim):
        """
        Checks that enable_comp sends a single write command to the FIFO.
        """
        self.assertTrue(sim.get(self.dut.fifo.empty))
        await self.host.enable_comp(laser0=True, laser1=False, polygon=False)
        await self.assert_fifo_written(1)

    @async_test_case
    async def test_fifo_not_empty_after_spline_move(self, sim):
        "Check spline move instruction writes the expected number of words to the FIFO"
        self.assertTrue(sim.get(self.dut.fifo.empty))
        cfg = self.hdl_cfg
        coeff = [randint(0, 10)] * cfg.motors * cfg.pol_degree
        await self.host.spline_move(1000, coeff)
        await self.assert_fifo_written(cfg.words_move)

    @async_test_case
    async def test_pin_state_reflects_random_inputs(self, sim):
        """Test that the pin state reflects randomly set digital inputs correctly."""

        async def test_pins():
            pin_keys = list(self.host.cfg.motor_cfg["steps_mm"].keys()) + [
                "photodiode_trigger",
                "synchronized",
            ]

            # Generate a random state dictionary
            expected_state = {key: randint(0, 1) for key in pin_keys}

            # Pack values into a bitfield (LSB = first key, MSB = last)
            bit_values = [expected_state[key] for key in reversed(pin_keys)]
            bitfield = int("".join(map(str, bit_values)), 2)

            # Apply to simulated pin_state and tick
            sim.set(self.dut.pin_state, bitfield)
            await sim.tick()

            # Retrieve state from host and filter for comparison
            actual_state = await self.host.fpga_state
            actual_state = {k: actual_state[k] for k in pin_keys}

            self.assertDictEqual(expected_state, actual_state)

        await test_pins()
        await test_pins()

    @async_test_case
    async def test_parser_can_be_disabled(self, sim):
        """
        Disable FIFO parser and verify FPGA state reflects the change.
        """
        await self.host.set_parsing(False)
        self.assertFalse(sim.get(self.dut.parse))
        self.assertFalse((await self.host.fpga_state)["parsing"])

    @async_test_case
    async def test_error_flag_set_on_invalid_instruction(self, sim):
        """
        Send invalid write instruction and verify FPGA sets error flag.
        """
        command = [Spi.Commands.write] + [0] * Spi.word_bytes
        await self.host.send_command(command)
        self.assertTrue((await self.host.fpga_state)["error"])

    @async_test_case
    async def test_fifo_reports_full_after_max_writes(self, sim):
        """
        Fill instruction FIFO until full and verify mem_full flag is set.
        """
        self.assertTrue(sim.get(self.dut.fifo.empty))
        self.assertFalse((await self.host.fpga_state)["mem_full"])

        try:
            for _ in range(self.hdl_cfg.mem_depth):
                await self.host.spline_move(1000, [1] * self.hdl_cfg.motors)
        except Memfull:
            pass
        self.assertTrue((await self.host.fpga_state)["mem_full"])


class TestDispatcher(SPIGatewareTestCase):
    plf_cfg = PlatformConfig(test=True)
    FRAGMENT_UNDER_TEST = Dispatcher
    FRAGMENT_ARGUMENTS = {"plf_cfg": plf_cfg}

    async def initialize_signals(self, sim):
        self.host = TestHost()
        self.sim = sim
        self.dut.spi = self.dut.parser.spi_command.spi
        self.host.spi_exchange_data = self.spi_exchange_data
        sim.set(self.dut.spi.cs, 0)
        await sim.tick()

    async def wait_complete(self, max_cycles=100) -> None:
        """Block until the dispatcher is fully idle."""
        sim = self.sim
        current_cycle = 0
        while sim.get(self.dut.busy) or current_cycle < max_cycles:
            if sim.get(self.dut.pol.busy):
                current_cycle = 0
            else:
                current_cycle += 1
            await sim.tick()

    @async_test_case
    async def test_memfull(self, sim):
        """
        Fill the command FIFO until it reports *mem_full*, then enable parsing
        and verify that

        * the fifo drains,
        * the *mem_full* flag clears,
        * **no error** flag is raised.
        """
        hdl_cfg = self.plf_cfg.hdl_cfg
        # should fill the memory as move instruction is
        # larger than the memdepth
        self.assertFalse((await self.host.fpga_state)["mem_full"])
        self.host.spi_tries = 1
        await self.host.set_parsing(False)
        try:
            for _ in range(hdl_cfg.mem_depth):
                await self.host.spline_move(1000, [1] * hdl_cfg.motors)
        except Memfull:
            pass
        self.assertTrue((await self.host.fpga_state)["mem_full"])
        await self.host.set_parsing(True)
        # data should now be processed from sram and empty become 1
        await self.wait_until(~self.dut.parser.fifo.empty)
        # 2 clocks needed for error to propagate
        await self.advance_cycles(2)
        self.assertFalse((await self.host.fpga_state)["error"])

    @async_test_case
    async def test_photodiode_trigger(self, sim):
        """verify you can receive photodiode trigger

        Photodiode trigger simply checks wether the photodiode
        has been triggered for each cycle.
        The photodiode is triggered by the simdiode.
        """
        facet_ticks = self.plf_cfg.laser_timing["facet_ticks"]
        await self.host.set_parsing(False)
        await self.host.enable_comp(laser0=True, polygon=True)
        await self.advance_cycles(facet_ticks * 2)
        self.assertFalse(sim.get(self.dut.lh.photodiode_t))
        # not triggered as laser and polygon not on
        await self.host.set_parsing(True)
        val = (await self.host.fpga_state)["photodiode_trigger"]
        await self.advance_cycles(facet_ticks * 2)
        self.assertTrue(sim.get(self.dut.lh.photodiode_t))
        self.assertTrue(val)

    @async_test_case
    async def test_write_pin_instruction(self, sim):
        """
        Send one *write_pin* instruction and verify the pin states recorded
        inside the laser-head simulator.
        """
        await self.host.enable_comp(
            laser0=True, laser1=False, polygon=True, synchronize=1, singlefacet=1
        )
        # wait till instruction is received
        await self.wait_until(~self.dut.parser.fifo.empty)
        await sim.tick()
        self.assertFalse((await self.host.fpga_state)["error"])
        self.assertTrue(sim.get(self.dut.lh.lh_rec.lasers[0]))
        self.assertFalse(sim.get(self.dut.lh.lh_rec.lasers[1]))
        self.assertTrue(sim.get(self.dut.lh.lh_rec.en))

        # NOT tested, these signals are not physical and not exposed via
        # laserheadpins
        # self.assertEqual(sim.get(self.dut.laserheadpins.synchronize), 1)
        # self.assertEqual(sim.get(self.dut.laserheadpins.singlefacet), 1)

    @async_test_case
    async def test_invalid_write_sets_error_flag(self, sim):
        """
        Push an illegal opcode into the parser FIFO and verify that
        the FPGA status register raises *error*.
        """
        fifo = self.dut.parser.fifo
        self.assertFalse((await self.host.fpga_state)["error"])
        # write illegal byte to queue and commit
        sim.set(fifo.write_data, 0xAA)
        await self.pulse(fifo.write_en)
        await self.pulse(fifo.write_commit)
        self.assertFalse(sim.get(fifo.empty))
        await self.wait_until(~self.dut.parser.fifo.empty)
        await self.advance_cycles(2)
        self.assertTrue((await self.host.fpga_state)["error"])

    @async_test_case
    async def test_movereceipt(self, sim, ticks=10_000):
        """
        Send one `SPLINE_MOVE` command and verify that

        tick limit & polynomial coefficient are transferred intact,
        the internal polynomial counters are correct.
        """
        hdl_cfg = self.plf_cfg.hdl_cfg
        motors = hdl_cfg.motors
        pol_degree = hdl_cfg.pol_degree
        coeff = [randint(-10, 10)] * motors * pol_degree
        await self.host.spline_move(ticks, coeff)
        # wait till instruction is received
        await self.wait_until(self.dut.pol.start)
        await sim.tick()
        await self.wait_until(~self.dut.pol.busy)

        # confirm receipt tick limit of segment
        self.assertEqual(sim.get(self.dut.pol.tick_limit), ticks)

        # confirm receipt coefficients
        for motor in range(motors):
            for coef in range(pol_degree):
                indx = motor * pol_degree + coef
                self.assertEqual(sim.get(self.dut.pol.coeff[indx]), coeff[indx])

        for motor in range(motors):
            cnt = 0
            for degree in range(pol_degree):
                indx = motor * pol_degree + degree
                cnt += ticks ** (degree + 1) * coeff[indx]
            self.assertEqual(sim.get(self.dut.pol.cntrs[motor * pol_degree]), cnt)

    @async_test_case
    async def test_home(self, sim):
        """
        Homing should

        1. detect asserted limit switches for every axis that is asked to home,
        2. drive those axes to the switch, clear the busy flag, and
        3. reset the position register to *zero* for each homed axis.
        """
        motors = self.plf_cfg.hdl_cfg.motors
        self.host._position = np.array([0.1] * motors)
        for i in range(motors):
            sim.set(self.dut.pol.steppers[i].limit, 1)
        await sim.tick()
        self.assertTrue(sim.get(self.dut.parser.pin_state[0]))
        await self.host.home_axes(
            axes=np.array([1] * motors),
            speed=None,
            displacement=-0.1,
        )
        assert_array_equal(
            self.host._position,
            np.array([0] * motors),
            err_msg="All axes should read position 0 after homing",
        )

    # @async_test_case
    # async def test_ptpmove(self, sim, steps=None, ticks=30_000):
    #     """verify point to point move

    #     If ticks is longer than tick limit the moves is broken up.
    #     If the number of instruction is larger than memdepth it
    #     also test blocking behaviour.
    #     """
    #     hdl_cfg = self.plf_cfg.hdl_cfg
    #     # TODO: remove this fix
    #     if steps is None:
    #         steps = [800] * hdl_cfg.motors

    #     mm = -np.array(steps) / np.array(
    #         list(self.plf_cfg.motor_cfg["steps_mm"].values())
    #     )
    #     time = ticks / hdl_cfg.motor_freq
    #     speed = np.abs(mm / time)
    #     await self.host.gotopoint(mm.tolist(), speed.tolist())
    #     # TODO: FOUT HIER!?
    #     await self.wait_complete()
    #     # if 76.3 steps per mm then 1/76.3 = 0.013 is max resolution
    #     assert_array_almost_equal(await self.host.position, mm, decimal=1)

    #     # TODO: they are not symmetric! if start with mm does not work
    #     mm = -mm
    #     await self.host.gotopoint(mm.tolist(), speed.tolist(), absolute=False)
    #     await self.wait_complete()
    #     assert_array_almost_equal(
    #         await self.host.position, np.zeros(hdl_cfg.motors), decimal=1
    #     )

    @async_test_case
    async def test_writeline(self, sim, num_lines=20, steps_line=0.5):
        """
        Queue a forward scan-pass followed by a return pass and verify that

        1. the `synchronized` flag rises when scan-lines are queued,
        2. the axis orthogonal to laser scan line moves by the expected distance,
        3. the axis returns to its start position after the return pass,
        4. no error flag is raised.
        """
        host = self.host
        laz_tim = self.plf_cfg.laser_timing
        motor_cfg = self.plf_cfg.motor_cfg
        for _ in range(num_lines):
            await host.write_line([1] * laz_tim["scanline_length"], steps_line, 0)
        await host.write_line([])
        self.assertTrue((await self.host.fpga_state)["synchronized"])
        await self.wait_until(self.dut.parser.fifo.empty)
        steps_mm = motor_cfg["steps_mm"][motor_cfg["orth2lsrline"]]
        decimals = int(np.log10(steps_mm))
        dist = num_lines * steps_line / steps_mm
        idx = list(motor_cfg["steps_mm"].keys()).index(motor_cfg["orth2lsrline"])
        # TODO: the x position changes as well!?
        assert_array_almost_equal(-dist, (await host.position)[idx], decimal=decimals)
        for _ in range(num_lines):
            await host.write_line([1] * laz_tim["scanline_length"], steps_line, 1)
        await host.write_line([])
        await host.enable_comp(synchronize=False)
        await self.wait_until(self.dut.parser.fifo.empty)
        # TODO: the engine should return to same position
        assert_array_almost_equal(0, (await host.position)[idx], decimal=decimals)
        fpga_state = await host.fpga_state
        self.assertFalse(fpga_state["synchronized"])
        self.assertFalse(fpga_state["error"])
