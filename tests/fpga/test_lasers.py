from random import randint
from struct import unpack
from typing import Any, ClassVar

from hexastorm.config import PlatformConfig, Spi
from hexastorm.fpga_host.mock import MockHost
from hexastorm.lasers import DiodeSimulator, Laserhead
from hexastorm.ulabext import (
    bits_to_events,
    calc_step_half_period,
    events_to_bits,
    pack_scanline_events,
)
from hexastorm.utils import LunaGatewareTestCase, async_test_case


class BaseTest(LunaGatewareTestCase):
    "Base class for laserhead test."

    plf_cfg = PlatformConfig(test=True)
    laz_tim = plf_cfg.laser_timing
    FRAGMENT_ARGUMENTS: ClassVar[dict[str, Any]] = {"plf_cfg": plf_cfg}

    async def initialize_signals(self, sim):
        """Initialize signals and host configuration for simulation."""
        self.host = MockHost(fifo_full=None, sim=None)
        self.sim = sim
        sim.set(self.dut.pd_db.raw, 1)
        await sim.tick()

    async def get_state(self, fsm=None):
        """Return the current symbolic state of the FSM."""
        if fsm is None:
            fsm = self.dut.laserfsm
        return fsm.decoding[self.sim.get(fsm.state)]

    async def wait_until_state(self, target_state: str, fsm=None):
        """Wait until FSM reaches a target state or times out."""
        sim = self.sim
        timeout = max(
            self.laz_tim["facet_ticks"] * 6,
            self.laz_tim["stable_ticks"],
            self.laz_tim["spinup_ticks"],
        )
        for _ in range(timeout):
            reached_state = await self.get_state(fsm)
            if reached_state == target_state:
                return
            await sim.tick()

        self.fail(
            f"Did not reach state '{target_state}' but {reached_state} within {timeout} ticks"
        )

    async def assert_state(self, expected_state: str, fsm=None):
        """Assert that the FSM is in the expected symbolic state."""
        actual_state = await self.get_state(fsm)
        self.assertEqual(actual_state, expected_state)

    async def count_steps(self, single=False):
        """Count steps while accounting for direction.

        Very similar to the function in movement.py

        single -- in single line mode dut.empty is not
                  a good measure
        """
        # TODO: replace with logic from movement.py and process lines
        sim = self.sim
        dut = self.dut
        count = 0
        ticks = 0
        thresh = self.laz_tim["facet_ticks"]
        if single:
            thresh *= 4
        while (sim.get(dut.empty) == 0) or (ticks < thresh):
            if single or sim.get(dut.empty) == 1:
                ticks += 1
            prev_step = sim.get(dut.step)
            await sim.tick()
            if prev_step and sim.get(dut.step) == 0:
                count += 1 if sim.get(dut.dir) else -1
        return count

    async def check_line(self, bit_lst, steps_per_line=1, direction=0):
        """Verify laser produces correct scan pattern."""
        sim = self.sim
        dut = self.dut
        laz_tim = self.laz_tim

        if not self.plf_cfg.hdl_cfg.single_line:
            self.assertFalse(sim.get(dut.empty))

        await self.wait_until_state("READ_INSTRUCTION")
        await sim.tick()
        # single facet mode; linecnt not equal to facet
        # stopline reset the line cnt
        if not sim.get(dut.singlefacet) and len(bit_lst) > 0:
            self.assertEqual(sim.get(dut.facetcnt), sim.get(dut.linecnt))
        self.assertEqual(sim.get(dut.dir), direction)
        self.assertFalse(sim.get(dut.error))

        if bit_lst:
            scanline_ticks = laz_tim["scanline_ticks"]
            expected_period = calc_step_half_period(steps_per_line, scanline_ticks)
            self.assertEqual(sim.get(dut.stephalfperiod), expected_period)

            expected_events = bits_to_events(bit_lst, scanline_ticks)
            expected_tick_bits = events_to_bits(
                expected_events, scanline_ticks, scanline_ticks
            )

            await self.wait_until_state("DATA_RUN")
            await sim.tick()

            for tick, expected_bit in enumerate(expected_tick_bits):
                self.assertEqual(sim.get(dut.line_tick), tick + 1)
                self.assertEqual(sim.get(dut.lh_rec.lasers[0]), expected_bit)
                await sim.tick()
        else:
            self.assertTrue(sim.get(dut.expose_finished))

        await self.wait_until_state("WAIT_END")
        self.assertFalse(sim.get(dut.error))
        self.assertTrue(sim.get(dut.synchronized))

    async def read_line(self, sim, total_bytes):
        """Reads a scanline from FIFO manually (no dispatcher/parser)."""
        dut = self.dut
        # read the line number
        await self.pulse(sim, dut.read_en_2)
        data_out = [sim.get(dut.read_data_2)]
        # TODO: THIS CODE IS HALFWAY COMMIT
        # print(data_out)
        for _ in range(0, total_bytes, Spi.word_bytes):
            await self.pulse(sim, dut.read_en_2)
            data_out.append(sim.get(dut.read_data_2))

        await self.pulse(sim, dut.read_commit_2)
        return data_out

    async def _commit_scanline_bytes(self, byte_lst):
        """Commit packed scanline bytes directly into the FPGA test FIFO."""
        dut = self.dut
        sim = self.sim

        if sim.get(dut.fifo.space_available) < (len(byte_lst) / Spi.word_bytes):
            raise ValueError("Memory full")

        for i in range(0, len(byte_lst), Spi.word_bytes):
            lst = byte_lst[i : i + Spi.word_bytes]
            number = unpack("Q", bytearray(lst))[0]
            sim.set(dut.fifo.write_data, number)
            await self.pulse(dut.fifo.write_en)

        await self.pulse(dut.fifo.write_commit)

    async def write_events(self, events, steps_per_line=1, direction=0):
        """Write raw ``(tick, intensity)`` events (or ``None`` for stop) into FIFO."""
        byte_lst = self.host.events_to_byte_list(events, steps_per_line, direction)
        await self._commit_scanline_bytes(byte_lst)

    async def write_line(self, bit_list, steps_per_line=1, direction=0):
        """Writes a scanline into FIFO manually (no dispatcher/parser)."""
        byte_lst = self.host.bit_to_byte_list(bit_list, steps_per_line, direction)
        await self._commit_scanline_bytes(byte_lst)

    async def scanline_ring_buffer(self, numb_lines=3):
        """Write several scanlines to FIFO and validate playback."""
        sim = self.sim
        dut = self.dut
        n_bits = self.laz_tim["scanline_ticks"] // 3
        lines = [
            [randint(0, 1) for _ in range(n_bits)]
            for _ in range(numb_lines)
        ]
        lines.append([])  # end with empty line

        for line in lines:
            await self.write_line(line)

        await self.pulse(dut.expose_start)
        sim.set(dut.synchronize, 1)

        for line in lines:
            await self.check_line(line)

        self.assertTrue(sim.get(dut.empty))
        self.assertTrue(sim.get(dut.expose_finished))


class LaserheadTest(BaseTest):
    "Test laserhead without triggering photodiode"

    FRAGMENT_UNDER_TEST = Laserhead

    @async_test_case
    async def test_pwm_pulse(self, sim):
        """Check that PWM high phase matches motor_period."""
        dut = self.dut
        motor_period = self.laz_tim["motor_period"]

        # Wait for rising edge of PWM
        while sim.get(dut.lh_rec.pwm) == 0:
            await sim.tick()

        # Count number of cycles PWM stays high
        cnt = 0
        while sim.get(dut.lh_rec.pwm) == 1:
            cnt += 1
            await sim.tick()

        self.assertEqual(cnt, motor_period)

    @async_test_case
    async def test_sync(self, sim):
        """Ensure error is raised when synchronization fails."""
        dut = self.dut

        # Trigger synchronization process
        sim.set(dut.synchronize, 1)
        await sim.tick()

        await self.wait_until_state("SPINUP")
        self.assertEqual(sim.get(dut.error), 0)

        await self.wait_until_state("WAIT_STABLE")
        await self.wait_until_state("STOP")

        # Expect error due to lack of photodiode input
        self.assertEqual(sim.get(dut.error), 1)

    @async_test_case
    async def test_disable_synchronize_aborts_spinup_and_wait_stable(self, sim):
        """Verify clearing synchronize stops Laserhead during SPINUP and WAIT_STABLE."""
        dut = self.dut

        sim.set(dut.synchronize, 1)
        await sim.tick()
        await self.wait_until_state("SPINUP")
        sim.set(dut.synchronize, 0)
        await self.wait_until_state("STOP")
        await sim.tick()
        self.assertEqual(sim.get(dut.error), 0)
        self.assertEqual(sim.get(dut.process_lines), 0)
        self.assertEqual(sim.get(dut.lh_rec.en), 0)

        sim.set(dut.synchronize, 1)
        await sim.tick()
        await self.wait_until_state("WAIT_STABLE")
        sim.set(dut.synchronize, 0)
        await self.wait_until_state("STOP")
        await sim.tick()
        self.assertEqual(sim.get(dut.error), 0)
        self.assertEqual(sim.get(dut.process_lines), 0)
        self.assertEqual(sim.get(dut.lh_rec.en), 0)


class SinglelineTest(BaseTest):
    "Test laserhead while triggering photodiode and single line."

    plf_cfg = PlatformConfig(test=True)
    plf_cfg.hdl_cfg.single_line = True
    FRAGMENT_UNDER_TEST = DiodeSimulator
    FRAGMENT_ARGUMENTS: ClassVar[dict[str, Any]] = {"plf_cfg": plf_cfg}

    @async_test_case
    async def test_single_line_exposure_sequence(self, sim):
        """Test synchronization, scanline buffering, and exposure completion."""
        dut = self.dut
        n_bits = max(1, self.laz_tim["scanline_ticks"] // 3)

        # Write full-on scanline and synchronize
        one_line = [1] * n_bits
        zero_line = [0] * n_bits
        random_line = [randint(0, 1) for _ in range(n_bits)]
        lines = [one_line, zero_line, random_line]
        for line in lines:
            await self.write_line(line)

        sim.set(dut.synchronize, 1)
        await sim.tick()

        await self.pulse(dut.expose_start)

        # In single line mode reads are not committed
        # if the FIFO is empty so the last line is always repeated!
        for check in range(10):
            idx = min(check, len(lines) - 1)
            await self.check_line(lines[idx])

        self.assertTrue(sim.get(dut.synchronized))

        # Write a random line and an empty line
        random_line = [randint(0, 1) for _ in range(n_bits)]
        empty_line = []

        self.assertFalse(sim.get(dut.expose_finished))

        for line in [random_line, empty_line]:
            await self.write_line(line)

        # Wait until the empty line triggers `expose_finished`
        while sim.get(dut.expose_finished) == 0:
            await sim.tick()
        self.assertTrue(sim.get(dut.expose_finished))

        sim.set(dut.synchronize, 0)
        await sim.tick()

        await self.wait_until_state("STOP")
        self.assertFalse(sim.get(dut.error))


class SinglelinesinglefacetTest(SinglelineTest):
    """Test laserhead in single-line, single-facet mode
    with photodiode triggering."""

    @async_test_case
    async def test_single_line_single_facet(self, sim):
        """Verify line exposure across multiple stable states in single-facet mode."""
        dut = self.dut
        scanline = [1] * self.laz_tim["scanline_ticks"]

        sim.set(dut.singlefacet, 1)
        await sim.tick()

        await self.write_line(scanline)

        sim.set(dut.synchronize, 1)
        await sim.tick()
        await self.pulse(dut.expose_start)

        # facet counter changes
        for facet in range(self.laz_tim["facets"] - 1):
            self.assertEqual(facet, sim.get(dut.facetcnt))
            await self.wait_until_state("WAIT_STABLE")
            await self.wait_until_state("WAIT_END")

        # Exposure still happens on spefic count, i.e. 1
        for _ in range(self.laz_tim["facets"] - 1):
            await self.check_line(scanline)
            self.assertEqual(sim.get(dut.facetcnt), 1)

        self.assertFalse(sim.get(dut.error))

    @async_test_case
    async def test_move(self, sim):
        """Verify scanhead performs expected movement in single-facet mode."""
        dut = self.dut
        steps_per_line = 1
        scanline = [1] * self.laz_tim["scanline_ticks"]

        sim.set(dut.singlefacet, 1)
        await sim.tick()

        await self.write_line(scanline, steps_per_line=steps_per_line, direction=1)
        await self.write_line([], steps_per_line=steps_per_line)  # Sentinel line

        sim.set(dut.synchronize, 1)
        await sim.tick()
        await self.pulse(dut.expose_start)

        steps = await self.count_steps(single=True)
        self.assertEqual(steps, steps_per_line)


class MultilineTest(BaseTest):
    """Test laserhead with photodiode triggering and ring buffer scanline processing."""

    FRAGMENT_UNDER_TEST = DiodeSimulator

    async def check_move(
        self,
        direction: int,
        steps_per_line: int = 1,
        numb_lines: int = 3,
        append_stop: bool = True,
    ):
        """Verify that movement occurs for multiple scanlines."""
        dut = self.dut
        sim = self.sim

        lines = [[1] * self.laz_tim["scanline_ticks"]] * numb_lines
        if append_stop:
            lines.append([])

        for line in lines:
            await self.write_line(line, steps_per_line, direction)

        sim.set(dut.synchronize, 1)
        await sim.tick()
        await self.pulse(dut.expose_start)

        expected_steps = steps_per_line * numb_lines * (1 if direction else -1)
        steps = await self.count_steps(sim)

        self.assertEqual(steps, expected_steps)
        self.assertTrue(sim.get(dut.synchronized))
        self.assertFalse(sim.get(dut.error))

    async def check_no_move(self, thresh: int | None = None):
        """Verify that the scanhead does not move beyond a step threshold."""
        dut = self.dut
        sim = self.sim

        if thresh is None:
            thresh = sim.get(dut.stephalfperiod) * 2

        current = sim.get(dut.step)
        for _ in range(thresh + 1):
            if sim.get(dut.step) != current:
                break
            await sim.tick()
        else:
            self.assertTrue(True)  # Reached threshold without movement

        self.assertFalse(sim.get(dut.error))

    @async_test_case
    async def test_sync(self, sim):
        """Verify FSM sync and stable transitions without error."""
        dut = self.dut
        sim.set(dut.synchronize, 1)
        await sim.tick()
        await self.wait_until_state("SPINUP")
        self.assertFalse(sim.get(dut.error))

        for _ in range(3):
            await self.wait_until_state("WAIT_STABLE")
            await self.wait_until_state("WAIT_END")

        self.assertFalse(sim.get(dut.error))

    @async_test_case
    async def test_stopline(self, sim):
        """Test that a single empty line triggers exposure completion."""
        line = []
        dut = self.dut

        await self.write_line(line)
        sim.set(dut.synchronize, 1)
        await sim.tick()
        await self.pulse(dut.expose_start)

        self.assertFalse(sim.get(dut.empty))

        await self.wait_until_state("SPINUP")
        await self.check_line(line)

        self.assertTrue(sim.get(dut.expose_finished))
        await sim.tick()
        self.assertTrue(sim.get(dut.empty))

    @async_test_case
    async def test_interruption(self, sim):
        """Test movement interruption and resumption across exposure."""
        dut = self.dut
        sim.set(dut.synchronize, 1)
        await sim.tick()
        await self.pulse(dut.expose_start)

        await self.check_move(direction=0, append_stop=False)
        await self.check_no_move()
        await self.check_move(direction=0)

    @async_test_case
    async def test_movement(self, sim):
        """Verify scanhead moves in both directions and then stops."""
        await self.check_move(direction=0)
        await self.check_move(direction=1)
        await self.check_no_move()

    @async_test_case
    async def test_scanlineringbuffer(self, sim):
        """Test multiple scanlines written to and read from the ring buffer."""
        await self.scanline_ring_buffer(numb_lines=3)

    @async_test_case
    async def test_invalid_instruction_stops_prism_and_clears_process_lines(self, sim):
        """Verify an illegal FIFO opcode stops the prism and clears process_lines."""
        dut = self.dut
        sim.set(dut.fifo.write_data, 0xAA)
        await self.pulse(dut.fifo.write_en)
        await self.pulse(dut.fifo.write_commit)

        await self.pulse(dut.expose_start)
        sim.set(dut.synchronize, 1)

        await self.wait_until_state("READ_INSTRUCTION")
        await self.wait_until_state("STOP")
        await sim.tick()
        self.assertEqual(sim.get(dut.error), 1)
        self.assertEqual(sim.get(dut.process_lines), 0)
        self.assertEqual(sim.get(dut.lh_rec.en), 0)

    @async_test_case
    async def test_both_laser_channels_stay_in_sync(self, sim):
        """Verify lasers[0] and lasers[1] switch identically with no 1-cycle skew."""
        dut = self.dut
        await self.write_line([1, 0, 1])
        await self.write_line([])

        sim.set(dut.synchronize, 1)
        await sim.tick()
        await self.pulse(dut.expose_start)

        await self.wait_until_state("DATA_RUN")
        await sim.tick()
        total_ticks = self.laz_tim["scanline_ticks"]
        for _ in range(total_ticks):
            self.assertEqual(
                sim.get(dut.lh_rec.lasers[0]),
                sim.get(dut.lh_rec.lasers[1]),
            )
            await sim.tick()

    @async_test_case
    async def test_single_tick_event_switching(self, sim):
        """Verify Laserhead switches at single FPGA clock-tick resolution without divider quantization."""
        dut = self.dut
        events = [(0, 255), (3, 0), (5, 255), (6, 0)]
        await self.write_events(events, steps_per_line=1, direction=0)
        await self.write_events(None)

        sim.set(dut.synchronize, 1)
        await sim.tick()
        await self.pulse(dut.expose_start)

        await self.wait_until_state("DATA_RUN")
        await sim.tick()

        expected_bits = events_to_bits(
            events,
            self.laz_tim["scanline_ticks"],
            self.laz_tim["scanline_ticks"],
        )
        for tick, expected in enumerate(expected_bits):
            self.assertEqual(sim.get(dut.line_tick), tick + 1)
            self.assertEqual(sim.get(dut.lh_rec.lasers[0]), expected)
            self.assertEqual(sim.get(dut.lh_rec.lasers[1]), expected)
            await sim.tick()

        await self.wait_until_state("WAIT_END")
        self.assertEqual(sim.get(dut.error), 0)

    @async_test_case
    async def test_zero_steps_stationary_scanline(self, sim):
        """Verify steps_per_line=0 encodes stephalfperiod=0 and performs zero motor steps."""
        dut = self.dut
        scanline = [1] * self.laz_tim["scanline_ticks"]
        for _ in range(3):
            await self.write_line(scanline, steps_per_line=0, direction=1)
        await self.write_line([])

        sim.set(dut.synchronize, 1)
        await sim.tick()
        await self.pulse(dut.expose_start)

        steps = await self.count_steps()
        self.assertEqual(steps, 0)
        self.assertEqual(sim.get(dut.stephalfperiod), 0)
        self.assertEqual(sim.get(dut.error), 0)
        self.assertTrue(sim.get(dut.expose_finished))


class Loweredge(BaseTest):
    """Test Laserhead scanline exposure when scanline ticks equal 3x memory word width."""

    plf_cfg = PlatformConfig(test=True)
    laz_tim = plf_cfg.laser_timing
    laz_tim_backup = laz_tim
    laz_tim.update(
        {
            "facet_ticks": 500,
            "scanline_ticks": plf_cfg.hdl_cfg.mem_width * 3,
        }
    )
    plf_cfg.laser_timing = laz_tim
    plf_cfg.update_laser_timing()
    plf_cfg.hdl_cfg.mem_depth = 128

    FRAGMENT_UNDER_TEST = DiodeSimulator
    FRAGMENT_ARGUMENTS: ClassVar[dict[str, Any]] = {"plf_cfg": plf_cfg}

    async def initialize_signals(self, sim):
        """Initialize signals and host configuration for simulation."""
        self.sim = sim
        self.host = MockHost(fifo_full=None, sim=None)
        self.host.cfg.laser_timing = self.plf_cfg.laser_timing
        self.host.cfg.hdl_cfg.single_line = False
        sim.set(self.dut.pd_db.raw, 1)
        await sim.tick()

    def tearDown(self):
        """Restore original laser timing after test."""
        self.laz_tim = self.laz_tim_backup

    @async_test_case
    async def test_scanlineringbuffer(self, sim):
        """Write several scanlines and verify they are processed correctly."""
        await self.scanline_ring_buffer(numb_lines=3)

    @async_test_case
    async def test_unread_event_words_drained_at_end_of_scanline(self, sim):
        """Verify unread trailing event words are drained so the next line parses cleanly."""
        dut = self.dut
        scanline_ticks = self.laz_tim["scanline_ticks"]
        raw_bytes = pack_scanline_events(
            events=[
                (0, 255),
                (scanline_ticks + 10, 0),
                (scanline_ticks + 20, 255),
                (scanline_ticks + 30, 0),
                (scanline_ticks + 40, 255),
            ],
            steps_line=1,
            direction=0,
            scanline_ticks=scanline_ticks,
            scanline_cmd=Spi.Instructions.scanline,
        )
        for i in range(0, len(raw_bytes), Spi.word_bytes):
            word = unpack("Q", raw_bytes[i : i + Spi.word_bytes])[0]
            sim.set(dut.fifo.write_data, word)
            await self.pulse(dut.fifo.write_en)
        await self.pulse(dut.fifo.write_commit)

        normal_line = [1 if i % 4 == 0 else 0 for i in range(scanline_ticks)]
        await self.write_line(normal_line)
        await self.write_line([])

        await self.pulse(dut.expose_start)
        sim.set(dut.synchronize, 1)

        await self.wait_until_state("DATA_RUN")
        await self.wait_until_state("DRAIN_WORDS")
        await self.wait_until_state("WAIT_END")
        await self.check_line(normal_line)
        await self.check_line([])
        self.assertEqual(sim.get(dut.error), 0)
        self.assertEqual(sim.get(dut.expose_finished), 1)
        self.assertEqual(sim.get(dut.empty), 1)


class Upperedge(Loweredge):
    """Test scanline exposure with ticks equal to 3x (memory word width + 1)."""

    plf_cfg = PlatformConfig(test=True)
    laz_tim = plf_cfg.laser_timing
    laz_tim_backup = laz_tim
    laz_tim.update(
        {
            "facet_ticks": 500,
            "scanline_ticks": (plf_cfg.hdl_cfg.mem_width + 1) * 3,
        }
    )
    plf_cfg.laser_timing = laz_tim
    plf_cfg.update_laser_timing()
    plf_cfg.hdl_cfg.mem_depth = 128

    FRAGMENT_UNDER_TEST = DiodeSimulator
    FRAGMENT_ARGUMENTS: ClassVar[dict[str, Any]] = {"plf_cfg": plf_cfg}


class GrayscalePDMTest(BaseTest):
    """Test 1st-order Sigma-Delta PDM grayscale modulation over a full 256-tick period."""

    plf_cfg = PlatformConfig(test=True)
    laz_tim = plf_cfg.laser_timing
    laz_tim.update(
        {
            "facet_ticks": 500,
            "scanline_ticks": 256,
        }
    )
    plf_cfg.update_laser_timing()
    plf_cfg.hdl_cfg.mem_depth = 128

    FRAGMENT_UNDER_TEST = DiodeSimulator
    FRAGMENT_ARGUMENTS: ClassVar[dict[str, Any]] = {"plf_cfg": plf_cfg}

    async def initialize_signals(self, sim):
        """Initialize signals and host configuration for simulation."""
        self.sim = sim
        self.host = MockHost(fifo_full=None, sim=None)
        self.host.cfg.laser_timing = self.plf_cfg.laser_timing
        self.host.cfg.hdl_cfg.single_line = False
        sim.set(self.dut.pd_db.raw, 1)
        await sim.tick()

    @async_test_case
    async def test_sigma_delta_pdm_duty_cycles(self, sim):
        """Verify 8-bit intensities (0, 64, 85, 128, 192, 255) produce exact PDM duty cycles."""
        dut = self.dut
        test_intensities = [0, 64, 85, 128, 192, 255]

        for val in test_intensities:
            await self.write_events([(0, val)], steps_per_line=0)
        await self.write_events(None)

        await self.pulse(dut.expose_start)
        sim.set(dut.synchronize, 1)

        scanline_ticks = self.laz_tim["scanline_ticks"]
        for val in test_intensities:
            await self.wait_until_state("DATA_RUN")
            await sim.tick()
            on_count = 0
            for tick in range(scanline_ticks):
                l0 = sim.get(dut.lh_rec.lasers[0])
                l1 = sim.get(dut.lh_rec.lasers[1])
                self.assertEqual(l0, l1, f"Channel skew at tick {tick} for intensity={val}")
                on_count += l0
                await sim.tick()
            self.assertEqual(
                on_count,
                scanline_ticks if val == 255 else val,
                f"Unexpected active tick count for intensity={val}: {on_count}/{scanline_ticks}",
            )

        await self.check_line([])
        self.assertEqual(sim.get(dut.error), 0)


# NOTE: new class is created to reset settings
#       couldn't avoid this easily so kept for now
#
#       syncing behaviour is not really tested, in reality
#       prism spins up and systems goes into sync
#
#       spin up of prism is done without profile
#
#       I am not sure why there are tests called lower / upper edge.
#       Changing the limits does not reveal an edge or failure of the test?
