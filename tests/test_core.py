from random import randint

import numpy as np
from numpy.testing import assert_array_almost_equal, assert_array_equal

from hexastorm.config import Spi, PlatformConfig
from hexastorm.utils import async_test_case
from hexastorm.spi import SPIGatewareTestCase
from hexastorm.controller import TestHost, Memfull
from hexastorm.core import SPIParser, Dispatcher
from hexastorm.platforms import Firestarter


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
        while sim.get(self.dut.fifo.empty) == 1:
            await sim.tick()
        self.assertEqual(sim.get(self.dut.fifo.empty), 0)
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
        while sim.get(self.dut.fifo.empty) == 1:
            await sim.tick()
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
        self.assertEqual(sim.get(self.dut.fifo.empty), 1)
        await self.host.enable_comp(laser0=True, laser1=False, polygon=False)
        await self.assert_fifo_written(1)

    @async_test_case
    async def test_fifo_not_empty_after_spline_move(self, sim):
        "Check spline move instruction writes the expected number of words to the FIFO"
        self.assertEqual(sim.get(self.dut.fifo.empty), 1)
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
        self.assertEqual(sim.get(self.dut.parse), 0)
        self.assertEqual((await self.host.fpga_state)["parsing"], 0)

    @async_test_case
    async def test_error_flag_set_on_invalid_instruction(self, sim):
        """
        Send invalid write instruction and verify FPGA sets error flag.
        """
        command = [Spi.Commands.write] + [0] * Spi.word_bytes
        await self.host.send_command(command)

        state = await self.host.fpga_state
        self.assertTrue(state["error"])

    @async_test_case
    async def test_fifo_reports_full_after_max_writes(self, sim):
        """
        Fill instruction FIFO until full and verify mem_full flag is set.
        """
        self.assertEqual(sim.get(self.dut.fifo.empty), 1)

        state = await self.host.fpga_state
        self.assertFalse(state["mem_full"])

        try:
            for _ in range(self.hdl_cfg.mem_depth):
                await self.host.spline_move(1000, [1] * self.hdl_cfg.motors)
        except Memfull:
            pass

        state = await self.host.fpga_state
        self.assertTrue(state["mem_full"])


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

    async def wait_complete(self):
        """helper method to completion"""
        cntr = 0
        sim = self.sim
        while sim.get(self.dut.busy) or cntr < 100:
            if sim.get(self.dut.pol.busy):
                cntr = 0
            else:
                cntr += 1
            await sim.tick()

    @async_test_case
    async def test_memfull(self, sim):
        """write move instruction until memory is full, enable parser
        and ensure there is no parser error.
        """
        # should fill the memory as move instruction is
        # larger than the memdepth
        self.assertEqual((await self.host.fpga_state)["mem_full"], False)
        await self.host.set_parsing(False)
        try:
            for _ in range(self.plf_cfg.hdl_cfg.mem_depth):
                await self.host.spline_move(1000, [1] * self.plf_cfg.hdl_cfg.motors)
        except Memfull:
            pass
        self.assertEqual((await self.host.fpga_state)["mem_full"], True)
        await self.host.set_parsing(True)
        # data should now be processed from sram and empty become 1
        while sim.get(self.dut.parser.fifo.empty) == 0:
            await sim.tick()
        # 2 clocks needed for error to propagate
        await sim.tick()
        await sim.tick()
        self.assertEqual((await self.host.fpga_state)["error"], False)

    @async_test_case
    async def test_readdiode(self, sim):
        """verify you can receive photodiode trigger

        Photodiode trigger simply checks wether the photodiode
        has been triggered for each cycle.
        The photodiode is triggered by the simdiode.
        """
        facet_ticks = self.plf_cfg.laser_timing["facet_ticks"]
        await self.host.set_parsing(False)
        await self.host.enable_comp(laser0=True, polygon=True)
        for _ in range(facet_ticks * 2):
            await sim.tick()
        self.assertEqual(sim.get(self.dut.lh_mod.photodiode_t), False)
        # not triggered as laser and polygon not on
        await self.host.set_parsing(True)
        val = (await self.host.fpga_state)["photodiode_trigger"]
        for _ in range(facet_ticks * 2):
            await sim.tick()
        self.assertEqual(sim.get(self.dut.lh_mod.photodiode_t), True)
        self.assertEqual(val, True)


#     @async_test_case
#     async def test_writepin(self, sim):
#         """verify homing procedure works correctly"""
#         await self.host.enable_comp(
#             laser0=True, laser1=False, polygon=True, synchronize=1, singlefacet=1
#         )
#         # wait till instruction is received
#         while sim.get(self.dut.parser.empty):
#             await sim.tick()
#         await sim.tick()
#         self.assertEqual((await self.host.fpga_state)["error"], False)
#         self.assertEqual(sim.get(self.dut.lh.laser0), 1)
#         self.assertEqual(sim.get(self.dut.lh.laser1), 0)
#         self.assertEqual(sim.get(self.dut.lh.en), 1)

#         # NOT tested, these signals are not physical and not exposed via
#         # laserheadpins
#         # self.assertEqual(sim.get(self.dut.laserheadpins.synchronize), 1)
#         # self.assertEqual(sim.get(self.dut.laserheadpins.singlefacet), 1)


#     @async_test_case
#     async def test_home(self, sim):
#         """verify homing procedure works correctly"""
#         self.host._position = np.array([0.1] * self.platform.motors)
#         for i in range(self.platform.motors):
#             sim.set(self.dut.steppers[i].limit, 1)
#         await sim.tick()
#         self.assertEqual(sim.get(self.dut.parser.pinstate[0]), 1)
#         await self.host.home_axes(
#             axes=np.array([1] * self.platform.motors),
#             speed=None,
#             displacement=-0.1,
#         )
#         assert_array_equal(self.host._position, np.array([0] * self.platform.motors))

#     @async_test_case
#     async def test_invalidwrite(self, sim):
#         """write invalid instruction and verify error is raised"""
#         fifo = self.dut.parser.fifo
#         self.assertEqual((await self.host.get_state())["error"], False)
#         # write illegal byte to queue and commit
#         sim.set(fifo.write_data, 0xAA)
#         await self.pulse(sim, fifo.write_en)
#         await self.pulse(sim, fifo.write_commit)
#         self.assertEqual(sim.get(self.dut.parser.empty), 0)
#         # data should now be processed from sram and empty become 1
#         while sim.get(self.dut.parser.empty) == 0:
#             await sim.tick()
#         # 2 clocks needed for error to propagate
#         await sim.tick()
#         await sim.tick()
#         self.assertEqual((await self.host.get_state())["error"], True)

#     @async_test_case
#     async def test_ptpmove(self, sim, steps=None, ticks=30_000):
#         """verify point to point move

#         If ticks is longer than tick limit the moves is broken up.
#         If the number of instruction is larger than memdepth it
#         also test blocking behaviour.
#         """
#         # TODO: remove this fix
#         if steps is None:
#             steps = [800] * self.platform.motors
#         mm = -np.array(steps) / np.array(list(self.platform.stepspermm.values()))
#         time = ticks / MOTORFREQ
#         speed = np.abs(mm / time)
#         await self.host.gotopoint(mm.tolist(), speed.tolist())
#         await self.wait_complete(sim)
#         # if 76.3 steps per mm then 1/76.3 = 0.013 is max resolution
#         assert_array_almost_equal(await self.host.position, mm, decimal=1)

#         # TODO: they are not symmetric! if start with mm does not work
#         mm = -mm
#         await self.host.gotopoint(mm.tolist(), speed.tolist(), absolute=False)
#         await self.wait_complete(sim)
#         assert_array_almost_equal(
#             await self.host.position, np.zeros(self.platform.motors), decimal=1
#         )

#     @async_test_case
#     async def test_movereceipt(self, sim, ticks=10_000):
#         "verify move instruction send over with spline move"
#         coeff = [randint(-10, 10)] * self.platform.motors * self.platform.poldegree
#         await self.host.spline_move(ticks, coeff)
#         # wait till instruction is received
#         while sim.get(self.dut.pol.start) == 0:
#             await sim.tick()
#         await sim.tick()
#         while sim.get(self.dut.pol.busy):
#             await sim.tick()
#         # confirm receipt tick limit and coefficients
#         self.assertEqual(sim.get(self.dut.pol.ticklimit), ticks)
#         for motor in range(self.platform.motors):
#             for coef in range(self.platform.poldegree):
#                 indx = motor * self.platform.poldegree + coef
#                 self.assertEqual(sim.get(self.dut.pol.coeff[indx]), coeff[indx])
#         for motor in range(self.platform.motors):
#             cnt = 0
#             for degree in range(self.platform.poldegree):
#                 indx = motor * self.platform.poldegree + degree
#                 cnt += ticks ** (degree + 1) * coeff[indx]
#             self.assertEqual(
#                 sim.get(self.dut.pol.cntrs[motor * self.platform.poldegree]), cnt
#             )

#     @async_test_case
#     async def test_writeline(self, sim, numblines=20, stepsperline=0.5):
#         "write line and see it is processed accordingly"
#         host = self.host
#         for _ in range(numblines):
#             await host.writeline(
#                 [1] * host.laser_params["BITSINSCANLINE"], stepsperline, 0
#             )
#         await host.writeline([])
#         self.assertEqual((await host.get_state())["synchronized"], True)
#         while sim.get(self.dut.parser.empty) == 0:
#             await sim.tick()
#         plat = host.platform
#         stepspermm = plat.stepspermm[plat.laser_axis]
#         decimals = int(np.log10(stepspermm))
#         dist = numblines * stepsperline / stepspermm
#         idx = list(plat.stepspermm.keys()).index(plat.laser_axis)
#         # TODO: the x position changes as well!?
#         assert_array_almost_equal(-dist, (await host.position)[idx], decimal=decimals)
#         for _ in range(numblines):
#             await host.writeline(
#                 [1] * host.laser_params["BITSINSCANLINE"], stepsperline, 1
#             )
#         await host.writeline([])
#         await host.enable_comp(synchronize=False)
#         while sim.get(self.dut.parser.empty) == 0:
#             await sim.tick()
#         # TODO: the engine should return to same position
#         assert_array_almost_equal(0, (await host.position)[idx], decimal=decimals)
#         self.assertEqual((await host.get_state())["synchronized"], False)
#         self.assertEqual((await host.get_state())["error"], False)
