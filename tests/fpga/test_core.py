from random import randint
from asyncio import TimeoutError
import numpy as np
from numpy.testing import assert_array_almost_equal, assert_array_equal

from hexastorm.config import Spi, PlatformConfig
from hexastorm.utils import async_test_case
from hexastorm.luna.spi import SPIGatewareTestCase
from hexastorm.fpga_host.mock import MockHost
from hexastorm.core import SPIParser, Dispatcher


class TestParser(SPIGatewareTestCase):
    hdl_cfg = PlatformConfig(test=True).hdl_cfg
    FRAGMENT_UNDER_TEST = SPIParser
    FRAGMENT_ARGUMENTS = {"hdl_cfg": hdl_cfg}

    async def initialize_signals(self, sim):
        self.host = MockHost(self.dut.fifo_full, sim)
        self.sim = sim
        self.dut.spi = self.dut.spi_command.spi
        self.host.spi_exchange_data = self.spi_exchange_data
        sim.set(self.dut.spi.cs, 0)
        await sim.tick()

    async def assert_fifo_written(self, size):
        """
        Waits for the FIFO to become non-empty and verifies that the expected
        number of bytes have been written.

        Args:
            expected_bytes (int): Number of bytes expected to be in the FIFO.
        """
        sim = self.sim
        # wait till fifo is filled
        await self.wait_until(~self.dut.fifo.empty)
        self.assertEqual(
            sim.get(self.dut.fifo.space_available),
            self.hdl_cfg.mem_depth - size,
        )

        # To get actual data
        # actual_data = []
        # for _ in range(size):
        #     sim.set(self.dut.fifo.read_en, 1)
        #     await self.sim.tick()
        #     read_byte_val = sim.get(self.dut.fifo.read_data)
        #     actual_data.append(read_byte_val)
        #     sim.set(self.dut.fifo.read_en, 0)

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

        for _ in range(self.hdl_cfg.mem_depth):
            try:
                await self.host.spline_move(1000, [1] * self.hdl_cfg.motors)
            except TimeoutError:
                break
        self.assertTrue(sim.get(self.dut.fifo_full))
        self.assertTrue((await self.host.fpga_state)["mem_full"])


class TestDispatcher(SPIGatewareTestCase):
    plf_cfg = PlatformConfig(test=True)
    FRAGMENT_UNDER_TEST = Dispatcher
    FRAGMENT_ARGUMENTS = {"plf_cfg": plf_cfg}

    async def initialize_signals(self, sim):
        self.sim = sim
        self.host = MockHost(self.dut.parser.fifo_full, sim)
        self.dut.spi = self.dut.parser.spi_command.spi
        self.host.spi_exchange_data = self.spi_exchange_data
        sim.set(self.dut.spi.cs, 0)

        # Initialize absolute position tracker for testing
        self.motors = self.plf_cfg.hdl_cfg.motors
        self.simulated_positions = [0] * self.motors
        self.prev_steps = [0] * self.motors

        await sim.tick()

    def _track_steps(self):
        """Synchronously sample step pins and update position."""
        steppers = self.dut.pol.steppers
        for i in range(self.motors):
            current_step = self.sim.get(steppers[i].step)
            # Detect rising edge
            if current_step == 1 and self.prev_steps[i] == 0:
                current_dir = self.sim.get(steppers[i].dir)
                if current_dir:
                    self.simulated_positions[i] += 1
                else:
                    self.simulated_positions[i] -= 1
            self.prev_steps[i] = current_step

    async def tick(self):
        """Advances the simulator while ensuring steps are tracked."""
        self._track_steps()
        await self.sim.tick()

    async def advance_cycles(self, cycles):
        """
        Crucial override: ANY delay invoked by LunaGatewareTestCase (like SPI communication)
        will route through here. This guarantees no steps are dropped during SPI delays.
        """
        for _ in range(cycles):
            await self.tick()

    async def _wait_until_signal(self, signal, target_state=1, timeout=10000):
        """Custom waiter to ensure steps are tracked while waiting."""
        cycles = 0
        while self.sim.get(signal) != target_state:
            await self.tick()
            cycles += 1
            if cycles >= timeout:
                raise TimeoutError("Timeout during _wait_until_signal")

    async def wait_complete(self, max_cycles=100) -> None:
        """Block until the dispatcher is fully idle."""
        current_cycle = 0
        while self.sim.get(self.dut.busy) or current_cycle < max_cycles:
            if self.sim.get(self.dut.pol.busy):
                current_cycle = 0
            else:
                current_cycle += 1
            await self.tick()

    def get_simulated_fpga_position_mm(self):
        """
        Converts the manually tracked step counts back into millimeters.
        Replaces the old `await self.host.fpga_position`.
        """
        steps_mm_dict = self.plf_cfg.motor_cfg["steps_mm"]
        steps_mm = list(steps_mm_dict.values())
        mm_positions = [
            self.simulated_positions[i] / steps_mm[i] for i in range(self.motors)
        ]
        return np.array(mm_positions)

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
        except TimeoutError:
            pass
        self.assertTrue((await self.host.fpga_state)["mem_full"])
        await self.host.set_parsing(True)
        # data should now be processed from sram and empty become 1
        await self.wait_until(self.dut.parser.fifo.empty)
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
    async def test_set_leds(self, sim):
        """
        Test setting individual LEDs via SPI without affecting other pin states.
        Verifies bits 5, 6, and 7 correspond correctly to blue, green, and red.
        """
        # Set some base hardware states to ensure they aren't overwritten
        await self.host.enable_comp(laser0=True, polygon=True)
        await self.advance_cycles(10)  # ensure parser consumes it

        # Turn on Blue and Red LEDs
        await self.host.set_leds(blue=True, green=False, red=True)
        await self.advance_cycles(10)  # ensure parser consumes it

        pins_val = sim.get(self.dut.pins)
        # Check LED bits
        self.assertEqual((pins_val >> 5) & 1, 1)  # Blue (bit 5)
        self.assertEqual((pins_val >> 6) & 1, 0)  # Green (bit 6)
        self.assertEqual((pins_val >> 7) & 1, 1)  # Red (bit 7)

        # Check that previous hardware states are maintained
        self.assertEqual((pins_val >> 0) & 1, 1)  # laser0 (bit 0)
        self.assertEqual((pins_val >> 2) & 1, 1)  # polygon (bit 2)

        # Modify only Green and Red, keep Blue as is
        await self.host.set_leds(green=True, red=False)
        await self.advance_cycles(10)

        pins_val = sim.get(self.dut.pins)
        self.assertEqual((pins_val >> 5) & 1, 1)  # Blue (unchanged, still 1)
        self.assertEqual((pins_val >> 6) & 1, 1)  # Green (now 1)
        self.assertEqual((pins_val >> 7) & 1, 0)  # Red (now 0)

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
        Homing done via diagnose pins of the TMC2209. Wether the home is reached is determined
        by micropython. The FPGA only relays the information.
        """
        motors = self.plf_cfg.hdl_cfg.motors
        for i in range(motors):
            sim.set(self.dut.pol.steppers[i].limit, 1)
        await sim.tick()
        self.assertTrue(sim.get(self.dut.parser.pin_state[0]))

    @async_test_case
    async def test_ptpmove(self, sim, steps=None, ticks=30_000):
        """verify point to point move

        If ticks is longer than tick limit the moves is broken up.
        If the number of instruction is larger than memdepth it
        also test blocking behaviour.
        """
        hdl_cfg = self.plf_cfg.hdl_cfg
        if steps is None:
            steps = [800] * hdl_cfg.motors

        mm = -np.array(steps) / np.array(
            list(self.plf_cfg.motor_cfg["steps_mm"].values())
        )
        time = ticks / hdl_cfg.motor_freq
        speed = np.abs(mm / time)

        # Reset software tracker before move
        self.simulated_positions = [0] * self.motors
        self.prev_steps = [sim.get(s.step) for s in self.dut.pol.steppers]

        await self.host.gotopoint(mm.tolist(), speed.tolist())
        # wait_complete will passively count the steps while blocking!
        await self.wait_complete()

        actual_pos = self.get_simulated_fpga_position_mm()
        # if 76.3 steps per mm then 1/76.3 = 0.013 is max resolution
        assert_array_almost_equal(actual_pos, mm, decimal=1)

        # Reverse move
        mm = -mm
        await self.host.gotopoint(mm.tolist(), speed.tolist(), absolute=False)
        await self.wait_complete()

        actual_pos_return = self.get_simulated_fpga_position_mm()
        assert_array_almost_equal(
            actual_pos_return, np.zeros(hdl_cfg.motors), decimal=1
        )

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

        # Reset software tracker before scan
        self.simulated_positions = [0] * self.motors
        self.prev_steps = [sim.get(s.step) for s in self.dut.pol.steppers]

        # Queue lines
        for _ in range(num_lines):
            await host.write_line([1] * laz_tim["scanline_length"], steps_line, 0)
        await host.write_line([])

        self.assertFalse((await self.host.fpga_state)["synchronized"])
        self.assertTrue((await self.host.fpga_state)["synchronized"])

        # We must use wait_complete here to ensure steps are actively tracked while waiting for FIFO to empty
        await self.wait_complete()

        steps_mm = motor_cfg["steps_mm"][motor_cfg["orth2lsrline"]]
        decimals = int(np.log10(steps_mm))
        dist = num_lines * steps_line / steps_mm
        idx = list(motor_cfg["steps_mm"].keys()).index(motor_cfg["orth2lsrline"])

        actual_pos = self.get_simulated_fpga_position_mm()
        assert_array_almost_equal(-dist, actual_pos[idx], decimal=decimals)

        # Return pass
        for _ in range(num_lines):
            await host.write_line([1] * laz_tim["scanline_length"], steps_line, 1)
        await host.write_line([])
        await host.enable_comp(synchronize=False)

        await self.wait_complete()

        actual_pos_return = self.get_simulated_fpga_position_mm()
        assert_array_almost_equal(0, actual_pos_return[idx], decimal=decimals)

        fpga_state = await host.fpga_state
        self.assertFalse(fpga_state["synchronized"])
        self.assertFalse(fpga_state["error"])

    @async_test_case
    async def test_facetticksperiod(self, sim, facet=2, ticksperiod=1200):
        """
        Turn on synchronization and polygon, then verify that facet number and ticks
        in facet can be received correctly.
        """
        host = self.host
        ticks_facet = self.plf_cfg.laser_timing["facet_ticks"]
        await host.enable_comp(synchronize=True)
        await self.wait_until(~self.dut.parser.fifo.empty)
        # timing is hard to get right involves SPI and laserhead,
        # test is roughly correct
        for idx, facet in enumerate([3, 0, 1]):
            if idx == 0:
                await self.advance_cycles(ticks_facet + 5)
            else:
                await self.advance_cycles(ticks_facet)
            [ticksperiod_rec, facet_rec] = await host.read_facet_ticks_and_id()
            self.assertEqual(facet_rec, facet)
            self.assertAlmostEqual(ticksperiod_rec, ticks_facet, delta=1)
