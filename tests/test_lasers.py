import unittest
from copy import deepcopy
from random import randint
from struct import unpack

from hexastorm.lasers import Laserhead, DiodeSimulator
from hexastorm.controller import TestHost
from hexastorm.config import Spi
from hexastorm.utils import LunaGatewareTestCase, async_test_case
from hexastorm.platforms import TestPlatform


class BaseTest(LunaGatewareTestCase):
    "Base class for laserhead test"

    platform = TestPlatform()
    laz_tim = platform.settings.laser_timing
    FRAGMENT_ARGUMENTS = {"platform": platform}

    async def initialize_signals(self, sim):
        self.host = TestHost()
        self.sim = sim
        sim.set(self.dut.photodiode, 1)
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
        for tick in range(timeout):
            if await self.get_state(fsm) == target_state:
                return
            await sim.tick()

        self.fail(f"Did not reach state '{target_state}' within {timeout} ticks")

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
            old = sim.get(dut.step)
            await sim.tick()
            if old and sim.get(dut.step) == 0:
                count += 1 if sim.get(dut.dir) else -1
        return count

    async def check_line(self, bit_lst, steps_per_line=1, direction=0):
        """Verify laser produces correct scan pattern."""
        sim = self.sim
        dut = self.dut
        laz_tim = self.laz_tim

        if not self.platform.hdl_cfg.single_line:
            self.assertFalse(sim.get(dut.empty))

        await self.wait_until_state("READ_INSTRUCTION")
        await sim.tick()

        self.assertEqual(sim.get(dut.dir), direction)
        self.assertFalse(sim.get(dut.error))

        if bit_lst:
            expected_period = steps_per_line * (laz_tim["scanline_length"] - 1) // 2
            self.assertEqual(sim.get(dut.stephalfperiod), expected_period)

            await self.wait_until_state("DATA_RUN")
            await sim.tick()

            for idx, bit in enumerate(bit_lst):
                self.assertEqual(sim.get(dut.lasercnt), laz_tim["laser_ticks"] - 1)
                self.assertEqual(sim.get(dut.scanbit), idx + 1)

                for _ in range(laz_tim["laser_ticks"]):
                    self.assertEqual(sim.get(dut.lasers[0]), bit)
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

    async def write_line(self, bit_list, steps_per_line=1, direction=0):
        """Writes a scanline into FIFO manually (no dispatcher/parser)."""
        byte_lst = self.host.bit_to_byte_list(bit_list, steps_per_line, direction)
        dut = self.dut
        sim = self.sim

        for i in range(0, len(byte_lst), Spi.word_bytes):
            lst = byte_lst[i : i + Spi.word_bytes]
            number = unpack("Q", bytearray(lst))[0]
            sim.set(dut.write_data, number)
            await self.pulse(dut.write_en)

        await self.pulse(dut.write_commit)

    async def scanline_ring_buffer(self, numb_lines=3):
        """Write several scanlines to FIFO and validate playback."""
        sim = self.sim
        dut = self.dut
        lines = [
            [randint(0, 1) for _ in range(self.laz_tim["scanline_length"])]
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
        while sim.get(dut.pwm) == 0:
            await sim.tick()

        # Count number of cycles PWM stays high
        cnt = 0
        while sim.get(dut.pwm) == 1:
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


class SinglelineTest(BaseTest):
    "Test laserhead while triggering photodiode and single line."

    platform = TestPlatform()
    platform.hdl_cfg.single_line = True
    FRAGMENT_UNDER_TEST = DiodeSimulator
    FRAGMENT_ARGUMENTS = {"platform": platform}

    @async_test_case
    async def test_single_line_exposure_sequence(self, sim):
        """Test synchronization, scanline buffering, and exposure completion."""
        dut = self.dut
        scanline_length = self.laz_tim["scanline_length"]

        # Write full-on scanline and synchronize
        lines = [[1] * scanline_length]
        for line in lines:
            await self.write_line(line)

        sim.set(dut.synchronize, 1)
        await sim.tick()

        await self.pulse(dut.expose_start)

        for _ in range(2):
            await self.check_line(line)

        self.assertTrue(sim.get(dut.synchronized))

        # Write a random line and an empty line
        random_line = [randint(0, 1) for _ in range(scanline_length)]
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


class SinglelinesinglefacetTest(BaseTest):
    """Test laserhead in single-line, single-facet mode
    with photodiode triggering."""

    platform = TestPlatform()
    platform.hdl_cfg.single_line = True
    FRAGMENT_UNDER_TEST = DiodeSimulator
    FRAGMENT_ARGUMENTS = {"platform": platform}

    @async_test_case
    async def test_single_line_single_facet(self, sim):
        """Verify line exposure across multiple stable states in single-facet mode."""
        dut = self.dut
        scanline = [1] * self.laz_tim["scanline_length"]

        sim.set(dut.singlefacet, 1)
        await sim.tick()

        await self.write_line(scanline)

        sim.set(dut.synchronize, 1)
        await sim.tick()
        await self.pulse(dut.expose_start)

        # Cycle through all non-final facets
        for facet in range(self.laz_tim["facets"] - 1):
            self.assertEqual(facet, sim.get(dut.facetcnt))
            await self.wait_until_state("WAIT_STABLE")
            await self.wait_until_state("WAIT_END")

        # Final line exposure
        for _ in range(self.laz_tim["facets"] - 1):
            await self.check_line(scanline)
            self.assertTrue(sim.get(dut.facetcnt))

        self.assertFalse(sim.get(dut.error))

    @async_test_case
    async def test_move(self, sim):
        """Verify scanhead performs expected movement in single-facet mode."""
        dut = self.dut
        steps_per_line = 1
        scanline = [1] * self.laz_tim["scanline_length"]

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

        lines = [[1] * self.laz_tim["scanline_length"]] * numb_lines
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

    async def check_no_move(self, thresh: int = None):
        """Verify that the scanhead does not move beyond a step threshold."""
        dut = self.dut
        sim = self.sim

        if thresh is None:
            thresh = sim.get(dut.stephalfperiod) * 2

        current = sim.get(dut.step)
        for count in range(thresh + 1):
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


# class Loweredge(BaseTest):
#     "Test Scanline of length MEMWIDTH"

#     platform = TestPlatform()
#     platform.hdl_cfg.single_line = True
#     laz_tim = platform.settings.laser_timing
#     laz_tim_backup = deepcopy(laz_tim)
#     FRAGMENT_UNDER_TEST = DiodeSimulator

#     async def initialize_signals(self, sim):
#         self.laz_tim["facet_ticks"] = 500
#         self.laz_tim["laser_ticks"] = 3
#         self.laz_tim["scanline_length"] = self.platform.hdl_cfg.mem_width
#         self.host = TestHost()
#         self.host.cfg.laser_timing = self.laz_tim
#         self.sim = sim
#         sim.set(self.dut.photodiode, 1)
#         await sim.tick()

#     def tearDown(self):
#         self.laz_tim = self.laz_tim_backup

#     @async_test_case
#     async def test_scanlineringbuffer(self, sim):
#         "write several scanlines and verify receival"
#         await self.scanline_ring_buffer(numb_lines=3)


# class Upperedge(Loweredge):
#     platform = TestPlatform()
#     FRAGMENT_UNDER_TEST = DiodeSimulator
#     dct = deepcopy(platform.laser_timing)
#     dct["TICKSINFACET"] = 500
#     dct["LASERTICKS"] = 3
#     dct["SINGLE_LINE"] = False
#     dct["BITSINSCANLINE"] = MEMWIDTH + 1
#     FRAGMENT_ARGUMENTS = {"platform": platform, "laser_var": dct}


# NOTE: new class is created to reset settings
#       couldn't avoid this easily so kept for now
#
#       syncing behaviour is not really tested, in reality
#       prism spins up and systems goes into sync
#
#       spin up of prism is done without profile
