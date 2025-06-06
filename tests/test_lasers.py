import unittest
from copy import deepcopy
from random import randint
from struct import unpack

from hexastorm.lasers import Laserhead, DiodeSimulator
import hexastorm.controller as controller
from hexastorm.config import MEMWIDTH, WORD_BYTES, params
from hexastorm.utils import LunaGatewareTestCase, async_test_case
from hexastorm.platforms import TestPlatform


class BaseTest(LunaGatewareTestCase):
    "Base class for laserhead test"

    async def initialize_signals(self, sim):
        sim.set(self.dut.photodiode, 1)
        self.host = controller.Host(self.platform)
        await sim.tick()

    async def getState(self, sim, fsm=None):
        if fsm is None:
            fsm = self.dut.laserfsm
        return fsm.decoding[sim.get(fsm.state)]

    async def count_steps(self, sim, single=False):
        """counts steps while accounting for direction
        single -- in single line mode dut.empty is not
                  a good measure
        Very similar to the function in movement.py
        """
        # TODO: replace with logic from movement.py and process lines
        count = 0
        dut = self.dut
        ticks = 0
        thresh = dut.dct["TICKSINFACET"]
        if single:
            thresh = 4 * thresh
        while (sim.get(dut.empty) == 0) or (ticks < thresh):
            if single or sim.get(dut.empty) == 1:
                ticks += 1
            old = sim.get(dut.step)
            await sim.tick()
            if old and sim.get(dut.step) == 0:
                count += 1 if sim.get(dut.dir) else -1
        return count

    async def waituntilState(self, sim, state, fsm=None):
        dut = self.dut
        timeout = max(
            dut.dct["TICKSINFACET"] * 6,
            dut.dct["STABLETICKS"],
            dut.dct["SPINUPTICKS"],
        )
        count = 0
        while await self.getState(sim, fsm) != state:
            await sim.tick()
            count += 1
            if count > timeout:
                print(f"Did not reach {state} in {timeout} ticks")
                self.assertTrue(count < timeout)

    async def assertState(self, sim, state, fsm=None):
        self.assertEqual(await self.getState(sim, fsm), state)

    async def checkline(self, sim, bitlst, stepsperline=1, direction=0):
        "it is verified wether the laser produces the pattern in bitlist"
        dut = self.dut
        if not dut.dct["SINGLE_LINE"]:
            self.assertEqual(sim.get(dut.empty), False)

        await self.waituntilState(sim, "READ_INSTRUCTION")
        await sim.tick()

        self.assertEqual(sim.get(dut.dir), direction)
        self.assertEqual(sim.get(dut.error), False)

        if len(bitlst):
            self.assertEqual(
                sim.get(dut.stephalfperiod),
                stepsperline * (dut.dct["BITSINSCANLINE"] - 1) // 2,
            )
            await self.waituntilState(sim, "DATA_RUN")
            await sim.tick()
            for idx, bit in enumerate(bitlst):
                self.assertEqual(sim.get(dut.lasercnt), dut.dct["LASERTICKS"] - 1)
                self.assertEqual(sim.get(dut.scanbit), idx + 1)
                for _ in range(dut.dct["LASERTICKS"]):
                    self.assertEqual(sim.get(dut.lasers[0]), bit)
                    await sim.tick()
        else:
            self.assertEqual(sim.get(dut.expose_finished), True)

        await self.waituntilState(sim, "WAIT_END")
        self.assertEqual(sim.get(dut.error), False)
        self.assertEqual(sim.get(dut.synchronized), True)

    async def read_line(self, sim, totalbytes):
        """reads line from fifo

        This is a helper function to allow testing of the module
        without dispatcher and parser
        """
        dut = self.dut
        # read the line number
        await self.pulse(sim, dut.read_en_2)
        data_out = [sim.get(dut.read_data_2)]
        # TODO: THIS CODE IS HALFWAY COMMIT
        # print(data_out)
        for _ in range(0, totalbytes, WORD_BYTES):
            await self.pulse(sim, dut.read_en_2)
            data_out.append(sim.get(dut.read_data_2))

        await self.pulse(sim, dut.read_commit_2)
        return data_out

    async def write_line(self, sim, bitlist, stepsperline=1, direction=0):
        """write line to fifo

        This is a helper function to allow testing of the module
        without dispatcher and parser
        """
        bytelst = self.host.bittobytelist(bitlist, stepsperline, direction)
        dut = self.dut

        for i in range(0, len(bytelst), WORD_BYTES):
            lst = bytelst[i : i + WORD_BYTES]
            number = unpack("Q", bytearray(lst))[0]
            sim.set(dut.write_data, number)
            await self.pulse(sim, dut.write_en)

        await self.pulse(sim, dut.write_commit)

    async def scanlineringbuffer(self, sim, numblines=3):
        "write several scanlines and verify receival"
        dut = self.dut
        lines = [
            [randint(0, 1) for _ in range(dut.dct["BITSINSCANLINE"])]
            for _ in range(numblines)
        ]
        lines.append([])  # sentinel line

        for line in lines:
            await self.write_line(sim, line)

        await self.pulse(sim, dut.expose_start)
        sim.set(dut.synchronize, 1)

        for line in lines:
            await self.checkline(sim, line)

        self.assertEqual(sim.get(dut.empty), True)
        self.assertEqual(sim.get(dut.expose_finished), True)


class LaserheadTest(BaseTest):
    "Test laserhead without triggering photodiode"

    platform = TestPlatform()
    FRAGMENT_UNDER_TEST = Laserhead
    FRAGMENT_ARGUMENTS = {"platform": platform}

    @async_test_case
    async def test_pwmpulse(self, sim):
        """pwm pulse generation test"""
        dut = self.dut
        dct = params(self.platform)

        # Wait for the pwm signal to go high
        while sim.get(dut.pwm) == 0:
            await sim.tick()

        # Count how many cycles the pwm stays high
        cnt = 0
        while sim.get(dut.pwm) == 1:
            cnt += 1
            await sim.tick()

        expected = int(dct["CRYSTAL_HZ"] / (dct["POLY_HZ"] * 6 * 2))
        self.assertEqual(cnt, expected)

    @async_test_case
    async def test_sync(self, sim):
        """error is raised if laser not synchronized"""
        dut = self.dut

        sim.set(dut.synchronize, 1)
        await sim.tick()

        await self.waituntilState(sim, "SPINUP")
        self.assertEqual(sim.get(dut.error), 0)

        await self.waituntilState(sim, "WAIT_STABLE")
        await self.waituntilState(sim, "STOP")

        self.assertEqual(sim.get(dut.error), 1)


class SinglelineTest(BaseTest):
    "Test laserhead while triggering photodiode and single line"

    platform = TestPlatform()
    laser_var = deepcopy(platform.laser_timing)
    laser_var["SINGLE_LINE"] = True
    FRAGMENT_UNDER_TEST = DiodeSimulator
    FRAGMENT_ARGUMENTS = {"platform": platform, "laser_var": laser_var}

    @async_test_case
    async def test_single_line(self, sim):
        dut = self.dut

        # Full line
        lines = [[1] * dut.dct["BITSINSCANLINE"]]
        for line in lines:
            await self.write_line(sim, line)

        sim.set(dut.synchronize, 1)
        await sim.tick()

        await self.pulse(sim, dut.expose_start)

        for _ in range(2):
            await self.checkline(sim, line)

        self.assertEqual(sim.get(dut.synchronized), True)

        # Two other lines (one random, one empty)
        lines = [[randint(0, 1) for _ in range(dut.dct["BITSINSCANLINE"])], []]
        self.assertEqual(sim.get(dut.expose_finished), 0)

        for line in lines:
            await self.write_line(sim, line)

        # The last line (empty) triggers exposure finished
        while sim.get(dut.expose_finished) == 0:
            await sim.tick()
        self.assertEqual(sim.get(dut.expose_finished), 1)

        sim.set(dut.synchronize, 0)
        await sim.tick()

        await self.waituntilState(sim, "STOP")
        self.assertEqual(sim.get(dut.error), False)


class SinglelinesinglefacetTest(BaseTest):
    """Test laserhead while triggering photodiode.

    Laserhead is in single line and single facet mode
    """

    platform = TestPlatform()
    laser_var = deepcopy(platform.laser_timing)
    laser_var["SINGLE_LINE"] = True
    FRAGMENT_UNDER_TEST = DiodeSimulator
    FRAGMENT_ARGUMENTS = {"platform": platform, "laser_var": laser_var}

    @async_test_case
    async def test_single_line_single_facet(self, sim):
        dut = self.dut

        sim.set(dut.singlefacet, 1)
        await sim.tick()

        lines = [[1] * dut.dct["BITSINSCANLINE"]]
        for line in lines:
            await self.write_line(sim, line)

        sim.set(dut.synchronize, 1)
        await sim.tick()

        await self.pulse(sim, dut.expose_start)

        for facet in range(self.platform.laser_timing["FACETS"] - 1):
            self.assertEqual(facet, sim.get(dut.facetcnt))
            await self.waituntilState(sim, "WAIT_STABLE")
            await self.waituntilState(sim, "WAIT_END")

        for _ in range(3):
            await self.checkline(sim, line)
            self.assertEqual(1, sim.get(dut.facetcnt))

        self.assertEqual(sim.get(dut.error), False)

    @async_test_case
    async def test_move(self, sim):
        dut = self.dut

        sim.set(dut.singlefacet, 1)
        await sim.tick()

        lines = [[1] * dut.dct["BITSINSCANLINE"]]
        stepsperline = 1

        for line in lines:
            await self.write_line(sim, line, stepsperline=stepsperline, direction=1)

        await self.write_line(sim, [], stepsperline=stepsperline)

        sim.set(dut.synchronize, 1)
        await sim.tick()

        await self.pulse(sim, dut.expose_start)

        steps = await self.count_steps(sim, single=True)
        self.assertEqual(steps, stepsperline)


class MultilineTest(BaseTest):
    "Test laserhead while triggering photodiode and ring buffer"

    platform = TestPlatform()
    FRAGMENT_UNDER_TEST = DiodeSimulator
    FRAGMENT_ARGUMENTS = {"platform": platform}

    async def checkmove(
        self, sim, direction, stepsperline=1, numblines=3, appendstop=True
    ):
        dut = self.dut
        lines = [[1] * dut.dct["BITSINSCANLINE"]] * numblines
        if appendstop:
            lines.append([])
        for line in lines:
            await self.write_line(sim, line, stepsperline, direction)
        sim.set(dut.synchronize, 1)
        await sim.tick()
        await self.pulse(sim, dut.expose_start)
        steps = await self.count_steps(sim)
        self.assertEqual(steps, stepsperline * numblines * (1 if direction else -1))
        self.assertTrue(sim.get(dut.synchronized))
        self.assertFalse(sim.get(dut.error))

    async def checknomove(self, sim, thresh=None):
        dut = self.dut
        if thresh is None:
            thresh = sim.get(dut.stephalfperiod) * 2

        current = sim.get(dut.step)
        count = 0
        res = False
        while sim.get(dut.step) == current:
            count += 1
            await sim.tick()
            if count > thresh:
                res = True
                break

        self.assertTrue(res)
        self.assertFalse(sim.get(dut.error))

    @async_test_case
    async def test_sync(self, sim):
        dut = self.dut
        sim.set(dut.synchronize, 1)
        await sim.tick()
        await self.waituntilState(sim, "SPINUP")
        self.assertEqual(sim.get(dut.error), 0)
        for _ in range(3):
            await self.waituntilState(sim, "WAIT_STABLE")
            await self.waituntilState(sim, "WAIT_END")
        self.assertEqual(sim.get(dut.error), 0)

    @async_test_case
    async def test_stopline(self, sim):
        line = []
        dut = self.dut
        await self.write_line(sim, line)
        sim.set(dut.synchronize, 1)
        await sim.tick()
        await self.pulse(sim, dut.expose_start)
        self.assertEqual(sim.get(dut.empty), 0)
        await self.waituntilState(sim, "SPINUP")
        sim.set(dut.synchronize, 1)
        await sim.tick()
        await self.checkline(sim, line)
        self.assertTrue(sim.get(dut.expose_finished))
        await sim.tick()
        await sim.tick()
        self.assertTrue(sim.get(dut.expose_finished))
        self.assertTrue(sim.get(dut.empty))

    @async_test_case
    async def test_interruption(self, sim):
        dut = self.dut
        sim.set(dut.synchronize, 1)
        await sim.tick()
        await self.pulse(sim, dut.expose_start)
        await self.checkmove(sim, 0, appendstop=False)
        await self.checknomove(sim)
        await self.checkmove(sim, 0)

    @async_test_case
    async def test_movement(self, sim):
        await self.checkmove(sim, direction=0)
        await self.checkmove(sim, direction=1)
        await self.checknomove(sim)

    @async_test_case
    async def test_scanlineringbuffer(self, sim):
        await self.scanlineringbuffer(sim, numblines=3)


class Loweredge(BaseTest):
    "Test Scanline of length MEMWIDTH"

    platform = TestPlatform()
    FRAGMENT_UNDER_TEST = DiodeSimulator

    dct = deepcopy(platform.laser_timing)
    dct["TICKSINFACET"] = 500
    dct["LASERTICKS"] = 3
    dct["SINGLE_LINE"] = False
    dct["BITSINSCANLINE"] = MEMWIDTH
    FRAGMENT_ARGUMENTS = {"platform": platform, "laser_var": dct}

    @async_test_case
    async def test_scanlineringbuffer(self, sim):
        "write several scanlines and verify receival"
        await self.scanlineringbuffer(sim, numblines=3)


class Upperedge(Loweredge):
    platform = TestPlatform()
    FRAGMENT_UNDER_TEST = DiodeSimulator
    dct = deepcopy(platform.laser_timing)
    dct["TICKSINFACET"] = 500
    dct["LASERTICKS"] = 3
    dct["SINGLE_LINE"] = False
    dct["BITSINSCANLINE"] = MEMWIDTH + 1
    FRAGMENT_ARGUMENTS = {"platform": platform, "laser_var": dct}


# NOTE: new class is created to reset settings
#       couldn't avoid this easily so kept for now
#
#       syncing behaviour is not really tested, in reality
#       prism spins up and systems goes into sync
#
#       spin up of prism is done without profile
