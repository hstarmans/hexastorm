import unittest
from random import randint

import numpy as np
from numpy.testing import assert_array_almost_equal, assert_array_equal

from hexastorm.utils import async_test_case
from hexastorm.spi import SPIGatewareTestCase
from hexastorm.constants import (
    MOTORFREQ, 
    wordsinscanline, 
    wordsinmove,
    COMMANDS,
    WORD_BYTES
)
from hexastorm.lasers import params
from hexastorm.controller import Host, Memfull
from hexastorm.core import SPIParser, Dispatcher
from hexastorm.platforms import TestPlatform

class TestParser(SPIGatewareTestCase):
    platform = TestPlatform()
    FRAGMENT_UNDER_TEST = SPIParser
    FRAGMENT_ARGUMENTS = {"platform": platform}

    async def initialize_signals(self, sim):
        self.host = Host(self.platform)
        self.host.spi_exchange_data = lambda data: self.spi_exchange_data(sim=sim, data=data)
        sim.set(self.dut.spi.cs, 0)
        await sim.tick()

    async def instruction_ready(self, sim, check):
        while sim.get(self.dut.empty) == 1:
            await sim.tick()
        self.assertEqual(sim.get(self.dut.empty), 0)
        self.assertEqual(
            sim.get(self.dut.fifo.space_available),
            self.platform.memdepth - check,
        )

    @async_test_case
    async def test_getposition(self, sim):
        decimals = 3
        position = [randint(-2000, 2000) for _ in range(self.platform.motors)]
        for idx, pos in enumerate(self.dut.position):
            sim.set(pos, position[idx])
        await sim.tick()
        lst = (await self.host.position).round(decimals)
        stepspermm = np.array(list(self.platform.stepspermm.values()))
        assert_array_equal(lst, (np.array(position) / stepspermm).round(decimals))

    @async_test_case
    async def test_writescanline(self, sim):
        await self.host.writeline([1] * self.platform.laser_var["BITSINSCANLINE"])
        while sim.get(self.dut.empty) == 1:
            await sim.tick()
        wordslaser = wordsinscanline(params(self.platform)["BITSINSCANLINE"])
        await self.instruction_ready(sim, wordslaser)

    @async_test_case
    async def test_lastscanline(self, sim):
        await self.host.writeline([])
        await self.instruction_ready(sim, 1)


    @async_test_case
    async def test_writepin(self, sim):
        "write move instruction and verify FIFO is no longer empty"
        self.assertEqual(sim.get(self.dut.empty), 1)
        await self.host.enable_comp(laser0=True, laser1=False, polygon=False)
        await self.instruction_ready(sim, 1)


    @async_test_case
    async def test_writemoveinstruction(self, sim):
        "write move instruction and verify FIFO is no longer empty"
        self.assertEqual(sim.get(self.dut.empty), 1)
        coeff = [randint(0, 10)] * self.platform.motors * self.platform.poldegree
        await self.host.spline_move(1000, coeff)
        words = wordsinmove(self.platform)
        await self.instruction_ready(sim, words)

    @async_test_case
    async def test_readpinstate(self, sim):
        """set pins to random state"""
        async def test_pins():
            keys = list(self.platform.stepspermm.keys()) + ["photodiode_trigger", "synchronized"]
            olddct = await self.host.get_state()
            olddct = {k: randint(0, 1) for k in keys}
            bitlist = list(olddct.values())[::-1]
            b = int("".join(str(i) for i in bitlist), 2)
            sim.set(self.dut.pinstate, b)
            await sim.tick()
            newdct = await self.host.get_state()
            newdct = {k: newdct[k] for k in keys}
            self.assertDictEqual(olddct, newdct)

        await test_pins()
        await test_pins()

    @async_test_case
    async def test_enableparser(self, sim):
        """enables SRAM parser via command and verifies status with
        different command"""
        await self.host.set_parsing(False)
        self.assertEqual(sim.get(self.dut.parse), 0)
        self.assertEqual((await self.host.get_state())["parsing"], 0)

    @async_test_case
    async def test_invalidwrite(self, sim):
        """write invalid instruction and verify error is raised"""
        command = [COMMANDS.WRITE] + [0] * WORD_BYTES
        await self.host.send_command(command)
        self.assertEqual((await self.host.get_state())["error"], True)

    @async_test_case
    async def test_memfull(self, sim):
        "write move instruction until memory is full"
        self.assertEqual(sim.get(self.dut.empty), 1)
        self.assertEqual((await self.host.get_state())["mem_full"], False)
        try:
            for _ in range(self.platform.memdepth):
                await self.host.spline_move(1000, [1] * self.platform.motors)
        except Memfull:
            pass
        self.assertEqual((await self.host.get_state())["mem_full"], True)

class TestDispatcher(SPIGatewareTestCase):
    platform = TestPlatform()
    FRAGMENT_UNDER_TEST = Dispatcher
    FRAGMENT_ARGUMENTS = {"platform": platform, "simdiode": True}


    async def initialize_signals(self, sim):
        self.host = Host(self.platform)
        self.host.spi_exchange_data = lambda data: self.spi_exchange_data(sim=sim, data=data)
        sim.set(self.dut.spi.cs, 0)
        await sim.tick()

    async def wait_complete(self, sim):
        """helper method to wait for completion"""
        cntr = 0
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
        self.assertEqual((await self.host.get_state())["mem_full"], False)
        await self.host.set_parsing(False)
        try:
            for _ in range(self.platform.memdepth):
                await self.host.spline_move(1000, [1] * self.platform.motors)
        except Memfull:
            pass
        self.assertEqual((await self.host.get_state())["mem_full"], True)
        await self.host.set_parsing(True)
        # data should now be processed from sram and empty become 1
        while sim.get(self.dut.parser.empty) == 0:
            await sim.tick()
        # 2 clocks needed for error to propagate
        await sim.tick()
        await sim.tick()
        self.assertEqual((await self.host.get_state())["error"], False)


    @async_test_case
    async def test_readdiode(self, sim):
        """verify you can receive photodiode trigger

        Photodiode trigger simply checks wether the photodiode
        has been triggered for each cycle.
        The photodiode is triggered by the simdiode.
        """
        await self.host.set_parsing(False)
        await self.host.enable_comp(laser0=True, polygon=True)
        for _ in range(self.dut.laserhead.dct["TICKSINFACET"] * 2):
            await sim.tick()
        self.assertEqual(sim.get(self.dut.laserhead.photodiode_t), False)
        # not triggered as laser and polygon not on
        await self.host.set_parsing(True)
        val = (await self.host.get_state())["photodiode_trigger"]
        for _ in range(self.dut.laserhead.dct["TICKSINFACET"] * 2):
            await sim.tick()
        self.assertEqual(sim.get(self.dut.laserhead.photodiode_t), True)
        self.assertEqual(val, True)


    @async_test_case
    async def test_writepin(self, sim):
        """verify homing procedure works correctly"""
        await self.host.enable_comp(
            laser0=True, laser1=False, polygon=True, synchronize=1, singlefacet=1
        )
        # wait till instruction is received
        while sim.get(self.dut.parser.empty):
            await sim.tick()
        await sim.tick()
        self.assertEqual((await self.host.get_state())["error"], False)
        self.assertEqual(sim.get(self.dut.laserheadpins.laser0), 1)
        self.assertEqual(sim.get(self.dut.laserheadpins.laser1), 0)
        self.assertEqual(sim.get(self.dut.laserheadpins.en), 1)
        # NOT tested, these signals are not physical and not exposed via 
        # laserheadpins
        # self.assertEqual(sim.get(self.dut.laserheadpins.synchronize), 1)
        # self.assertEqual(sim.get(self.dut.laserheadpins.singlefacet), 1)


    @async_test_case
    async def test_home(self, sim):
        """verify homing procedure works correctly"""
        self.host._position = np.array([0.1] * self.platform.motors)
        for i in range(self.platform.motors):
            sim.set(self.dut.steppers[i].limit, 1)
        await sim.tick()
        self.assertEqual(sim.get(self.dut.parser.pinstate[0]), 1)
        await self.host.home_axes(
            axes=np.array([1] * self.platform.motors),
            speed=None,
            displacement=-0.1,
        )
        assert_array_equal(self.host._position, np.array([0] * self.platform.motors))

    @async_test_case
    async def test_invalidwrite(self, sim):
        """write invalid instruction and verify error is raised"""
        fifo = self.dut.parser.fifo
        self.assertEqual((await self.host.get_state())["error"], False)
        # write illegal byte to queue and commit
        sim.set(fifo.write_data, 0xAA)
        await self.pulse(sim, fifo.write_en)
        await self.pulse(sim, fifo.write_commit)
        self.assertEqual(sim.get(self.dut.parser.empty), 0)
        # data should now be processed from sram and empty become 1
        while sim.get(self.dut.parser.empty) == 0:
            await sim.tick()
        # 2 clocks needed for error to propagate
        await sim.tick()
        await sim.tick()
        self.assertEqual((await self.host.get_state())["error"], True)

    @async_test_case
    async def test_ptpmove(self, sim, steps=None, ticks=30_000):
        """verify point to point move

        If ticks is longer than tick limit the moves is broken up.
        If the number of instruction is larger than memdepth it
        also test blocking behaviour.
        """
        # TODO: remove this fix
        if steps is None:
            steps = [800] * self.platform.motors
        mm = -np.array(steps) / np.array(list(self.platform.stepspermm.values()))
        time = ticks / MOTORFREQ
        speed = np.abs(mm / time)
        await self.host.gotopoint(mm.tolist(), speed.tolist())
        await self.wait_complete(sim)
        # if 76.3 steps per mm then 1/76.3 = 0.013 is max resolution
        assert_array_almost_equal(await self.host.position, mm, decimal=1)

        # TODO: they are not symmetric! if start with mm does not work
        mm = -mm
        await self.host.gotopoint(mm.tolist(), speed.tolist(), absolute=False)
        await self.wait_complete(sim)
        assert_array_almost_equal(await self.host.position, np.zeros(self.platform.motors), decimal=1)


    @async_test_case
    async def test_movereceipt(self, sim, ticks=10_000):
        "verify move instruction send over with spline move"
        coeff = [randint(-10, 10)] * self.platform.motors * self.platform.poldegree
        await self.host.spline_move(ticks, coeff)
        # wait till instruction is received
        while sim.get(self.dut.pol.start) == 0:
            await sim.tick()
        await sim.tick()
        while sim.get(self.dut.pol.busy):
            await sim.tick()
        # confirm receipt tick limit and coefficients
        self.assertEqual(sim.get(self.dut.pol.ticklimit), ticks)
        for motor in range(self.platform.motors):
            for coef in range(self.platform.poldegree):
                indx = motor * self.platform.poldegree + coef
                self.assertEqual(sim.get(self.dut.pol.coeff[indx]), coeff[indx])
        for motor in range(self.platform.motors):
            cnt = 0
            for degree in range(self.platform.poldegree):
                indx = motor * self.platform.poldegree + degree
                cnt += ticks ** (degree + 1) * coeff[indx]
            self.assertEqual(sim.get(self.dut.pol.cntrs[motor * self.platform.poldegree]), cnt)

    @async_test_case
    async def test_writeline(self, sim, numblines=20, stepsperline=0.5):
        "write line and see it is processed accordingly"
        host = self.host
        for _ in range(numblines):
            await host.writeline([1] * host.laser_params["BITSINSCANLINE"], stepsperline, 0)
        await host.writeline([])
        self.assertEqual((await host.get_state())["synchronized"], True)
        while sim.get(self.dut.parser.empty) == 0:
            await sim.tick()
        plat = host.platform
        stepspermm = plat.stepspermm[plat.laser_axis]
        decimals = int(np.log10(stepspermm))
        dist = numblines * stepsperline / stepspermm
        idx = list(plat.stepspermm.keys()).index(plat.laser_axis)
        # TODO: the x position changes as well!?
        assert_array_almost_equal(-dist, (await host.position)[idx], decimal=decimals)
        for _ in range(numblines):
            await host.writeline([1] * host.laser_params["BITSINSCANLINE"], stepsperline, 1)
        await host.writeline([])
        await host.enable_comp(synchronize=False)
        while sim.get(self.dut.parser.empty) == 0:
            await sim.tick()
        # TODO: the engine should return to same position
        assert_array_almost_equal(0, (await host.position)[idx], decimal=decimals)
        self.assertEqual((await host.get_state())["synchronized"], False)
        self.assertEqual((await host.get_state())["error"], False)
