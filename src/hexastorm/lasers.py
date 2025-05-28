import unittest
from copy import deepcopy
from random import randint
from struct import unpack

from amaranth import Elaboratable, Module, Signal
from luna.gateware.memory import TransactionalizedFIFO

from . import controller
from .constants import INSTRUCTIONS, MEMWIDTH, WORD_BYTES, params
from .utils import LunaGatewareTestCase, async_test_case
from .platforms import TestPlatform


class Laserhead(Elaboratable):
    """Controller of laser scanner with rotating mirror or prism

    I/O signals:
    O: synchronized   -- if true, laser is in sync and prism is rotating
    I: synchronize    -- activate synchronization
    I: singlefacet    -- activate single facet
    I: expose_start   -- start reading lines and exposing
    O: expose_finish  -- exposure is finished
    O: error          -- error signal
    O: lasers         -- laser pin
    O: pwm            -- pulse for scanner motor
    O: enable_prism   -- enable pin scanner motor
    I: photodiode     -- trigger for photodiode
    O: photodiode_t   -- high if photodiode triggered in this cycle
    O: photodiode_2   -- an attemp to add laser microscopy (not tested)
    O: read_commit    -- finalize read transactionalizedfifo
    O: read_en        -- enable read transactionalizedfifo
    I: read_data      -- read data from transactionalizedfifo, AKA busy
    I: empty          -- signal wether fifo is empty
    O: step           -- step signal
    O: direction      -- direction signal
    """

    def __init__(self, platform):
        """
        platform   -- pass test platform
        """
        self.platform = platform
        self.dct = params(platform)
        self.status = Signal()
        self.lasers = Signal(2)
        self.pwm = Signal()
        self.enable_prism = Signal()
        self.synchronize = Signal()
        self.synchronized = Signal()
        self.singlefacet = Signal()
        self.error = Signal()
        self.ticksinfacet = Signal(range(self.dct["TICKSINFACET"] * 2))
        self.photodiode = Signal()
        self.photodiode_t = Signal()
        self.photodiode_2 = Signal()
        # memory; read operations
        self.read_commit = Signal()
        self.read_en = Signal()
        self.read_data = Signal(MEMWIDTH)
        self.read_discard = Signal()
        self.empty = Signal()
        # memory; write operations
        self.write_commit_2 = Signal()
        self.write_en_2 = Signal()
        self.write_data_2 = Signal(MEMWIDTH)
        self.write_discard_2 = Signal()
        self.full = Signal()

        self.expose_finished = Signal()
        self.expose_start = Signal()
        self.step = Signal()
        self.dir = Signal()
        
        self.process_lines = Signal()

    def elaborate(self, platform):
        m = Module()
        dct = self.dct

        # Pulse generator for prism motor
        pwmcnt = Signal(range(dct["POLYPERIOD"]))
        # photodiode_triggered
        # TODO: not added to tests
        photodiodecnt = Signal(range(dct["TICKSINFACET"] * 2))
        triggered = Signal()
        with m.If(photodiodecnt < (dct["TICKSINFACET"] * 2 - 1)):
            with m.If(~self.photodiode):
                m.d.sync += triggered.eq(1)
            m.d.sync += photodiodecnt.eq(photodiodecnt + 1)
        with m.Else():
            m.d.sync += [
                self.photodiode_t.eq(triggered),
                photodiodecnt.eq(0),
                triggered.eq(0),
            ]

        # step generator, i.e. slowest speed is 1/(2^4-1)
        stephalfperiod = Signal(dct["BITSINSCANLINE"].bit_length() + 4)
        stepcnt = Signal.like(stephalfperiod)

        # TODO: this can be removed with the new driver!
        # pwm is always created but can be deactivated
        with m.If(pwmcnt == 0):
            m.d.sync += [
                self.pwm.eq(~self.pwm),
                pwmcnt.eq(dct["POLYPERIOD"] - 1),
            ]
        with m.Else():
            m.d.sync += pwmcnt.eq(pwmcnt - 1)

        # Laser FSM
        # stable thresh is changed, larger at start and then lowered
        stablethresh = Signal(range(dct["STABLETICKS"]))
        facetcnt = Signal(range(dct["FACETS"]))
        lasercnt = Signal(range(dct["LASERTICKS"]))
        scanbit = Signal(range(dct["BITSINSCANLINE"] + 1))
        tickcounter = Signal(
            range(max(dct["SPINUPTICKS"], dct["STABLETICKS"]))
        )
        scanlinenumber = Signal(range(255))
        photodiode = self.photodiode
        read_data = self.read_data
        write_data_2 = self.write_data_2
        write_new = Signal.like(write_data_2)
        read_old = Signal.like(read_data)
        readbit = Signal(range(MEMWIDTH))
        photodiode_d = Signal()
        lasers = self.lasers
        if self.platform.name == "Test":
            self.stephalfperiod = stephalfperiod
            self.tickcounter = tickcounter
            self.scanbit = scanbit
            self.lasercnt = lasercnt
            self.facetcnt = facetcnt

        # Exposure start detector
        expose_start_d = Signal()


        m.d.sync += expose_start_d.eq(self.expose_start)
        with m.If((expose_start_d == 0) & self.expose_start):
            m.d.sync += [self.process_lines.eq(1), self.expose_finished.eq(0)]

        with m.FSM(init="RESET") as laserfsm:
            with m.State("RESET"):
                m.d.sync += [self.error.eq(0), self.ticksinfacet.eq(0)]
                m.next = "STOP"
            with m.State("STOP"):
                m.d.sync += [
                    stablethresh.eq(dct["STABLETICKS"] - 1),
                    tickcounter.eq(0),
                    self.synchronized.eq(0),
                    self.enable_prism.eq(0),
                    readbit.eq(0),
                    facetcnt.eq(0),
                    scanbit.eq(0),
                    lasercnt.eq(0),
                    lasers.eq(0),
                ]
                with m.If(self.synchronize & (~self.error)):
                    # laser is off, photodiode cannot be triggered
                    with m.If(self.photodiode == 0):
                        m.d.sync += self.error.eq(1)
                        m.next = "STOP"
                    with m.Else():
                        m.d.sync += [self.error.eq(0), self.enable_prism.eq(1)]
                        m.next = "SPINUP"
            with m.State("SPINUP"):
                with m.If(tickcounter > dct["SPINUPTICKS"] - 1):
                    # turn on single channel
                    m.d.sync += [
                        self.lasers.eq(int("10", 2)),
                        tickcounter.eq(0),
                    ]
                    m.next = "WAIT_STABLE"
                with m.Else():
                    m.d.sync += tickcounter.eq(tickcounter + 1)
            with m.State("WAIT_STABLE"):
                m.d.sync += photodiode_d.eq(photodiode)
                with m.If(tickcounter >= stablethresh):
                    m.d.sync += self.error.eq(1)
                    m.next = "STOP"
                with m.Elif(~photodiode & ~photodiode_d):
                    m.d.sync += [tickcounter.eq(0), lasers.eq(0)]
                    with m.If(
                        (
                            tickcounter
                            > (dct["TICKSINFACET"] - 1) - dct["JITTERTICKS"]
                        )
                    ):
                        m.d.sync += [
                            self.synchronized.eq(1),
                            self.ticksinfacet.eq(tickcounter),
                        ]
                        with m.If(facetcnt == dct["FACETS"] - 1):
                            m.d.sync += facetcnt.eq(0)
                        with m.Else():
                            m.d.sync += facetcnt.eq(facetcnt + 1)
                        with m.If(self.singlefacet & (facetcnt > 0)):
                            m.next = "WAIT_END"
                        with m.Elif(self.empty | ~self.process_lines):
                            m.next = "WAIT_END"
                        with m.Else():
                            # TODO: 10 is too high, should be lower
                            thresh = min(
                                round(10.1 * dct["TICKSINFACET"]),
                                dct["STABLETICKS"],
                            )
                            m.d.sync += [
                                stablethresh.eq(thresh),
                                self.read_en.eq(1),
                            ]
                            m.next = "READ_INSTRUCTION"
                    with m.Else():
                        m.d.sync += self.synchronized.eq(0)
                        m.next = "WAIT_END"
                with m.Else():
                    m.d.sync += tickcounter.eq(tickcounter + 1)
            with m.State("READ_INSTRUCTION"):
                m.d.sync += [
                    self.read_en.eq(0),
                    tickcounter.eq(tickcounter + 1),
                ]
                with m.If(read_data[0:8] == INSTRUCTIONS.SCANLINE):
                    with m.If(scanlinenumber < 255):
                        m.d.sync += scanlinenumber.eq(scanlinenumber + 1)
                    with m.Else():
                        m.d.sync += scanlinenumber.eq(0)
                    m.d.sync += [
                        write_data_2.eq(scanlinenumber),
                        self.dir.eq(read_data[8]),
                        stephalfperiod.eq(read_data[9:]),
                    ]
                    m.next = "WAIT_FOR_DATA_RUN"
                with m.Elif(read_data == INSTRUCTIONS.LASTSCANLINE):
                    m.d.sync += [
                        self.expose_finished.eq(1),
                        self.read_commit.eq(1),
                        self.process_lines.eq(0),
                    ]
                    m.next = "WAIT_END"
                with m.Else():
                    m.d.sync += self.error.eq(1)
                    m.next = "READ_INSTRUCTION"
            with m.State("WAIT_FOR_DATA_RUN"):
                m.d.sync += [
                    tickcounter.eq(tickcounter + 1),
                    readbit.eq(0),
                    scanbit.eq(0),
                    lasercnt.eq(0),
                ]
                tickcnt_thresh = int(dct["START%"] * dct["TICKSINFACET"])
                assert tickcnt_thresh > 0
                with m.If(tickcounter >= tickcnt_thresh):
                    m.d.sync += [self.read_en.eq(1), self.write_en_2.eq(1)]
                    m.next = "DATA_RUN"
            with m.State("DATA_RUN"):
                m.d.sync += [tickcounter.eq(tickcounter + 1),
                             self.lasers[1].eq(self.lasers[0])]
                # NOTE:
                #      readbit is your current position in memory
                #      scanbit current byte position in scanline
                #      lasercnt used to pulse laser at certain freq
                with m.If(lasercnt == 0):
                    with m.If(stepcnt >= stephalfperiod):
                        m.d.sync += [self.step.eq(~self.step), stepcnt.eq(0)]
                    with m.Else():
                        m.d.sync += stepcnt.eq(stepcnt + 1)
                    with m.If(scanbit >= dct["BITSINSCANLINE"]):
                        m.d.sync += [
                            self.write_commit_2.eq(1),
                            self.lasers[0].eq(0),
                        ]
                        with m.If(dct["SINGLE_LINE"] & self.empty):
                            m.d.sync += self.read_discard.eq(1)
                        with m.Else():
                            m.d.sync += self.read_commit.eq(1)
                        m.next = "WAIT_END"
                    with m.Else():
                        m.d.sync += [
                            lasercnt.eq(dct["LASERTICKS"] - 1),
                            scanbit.eq(scanbit + 1),
                        ]
                        m.d.sync += write_new[0].eq(self.photodiode_2)
                        with m.If(readbit == 0):
                            m.d.sync += [
                                self.lasers[0].eq(read_data[0]),
                                read_old.eq(read_data >> 1),
                                self.read_en.eq(0),
                                self.write_en_2.eq(0),
                            ]
                        with m.Elif(readbit == MEMWIDTH - 1):
                            m.d.sync += [
                                write_data_2.eq(write_new),
                                self.lasers[0].eq(read_old[0]),
                            ]
                        with m.Else():
                            m.d.sync += self.lasers[0].eq(read_old[0])
                with m.Else():
                    m.d.sync += lasercnt.eq(lasercnt - 1)
                    # NOTE: read enable can only be high for 1 cycle
                    #       as a result this is done right before the "read"
                    with m.If(lasercnt == 1):
                        with m.If(readbit == 0):
                            m.d.sync += [readbit.eq(readbit + 1)]
                        # final read bit copy memory
                        # move to next address, i.e. byte, if end is reached
                        with m.Elif(readbit == MEMWIDTH - 1):
                            # If fifo is empty it will give errors later
                            # so it can be ignored here
                            # Only grab a new line if more than current
                            # is needed
                            # -1 as counting in python is different
                            with m.If(scanbit < (dct["BITSINSCANLINE"])):
                                m.d.sync += [
                                    self.read_en.eq(1),
                                    self.write_en_2.eq(1),
                                ]
                            m.d.sync += readbit.eq(0)
                        with m.Else():
                            m.d.sync += [
                                readbit.eq(readbit + 1),
                                read_old.eq(read_old >> 1),
                            ]
            with m.State("WAIT_END"):
                m.d.sync += [
                    tickcounter.eq(tickcounter + 1),
                    self.write_commit_2.eq(0),
                ]
                with m.If(dct["SINGLE_LINE"] & self.empty):
                    m.d.sync += self.read_discard.eq(0)
                with m.Else():
                    m.d.sync += self.read_commit.eq(0)
                
                # following lines can be used to test exposure width during
                # LaserHeadTest.test_stable
                # start_line = int(dct["START%"] * dct["TICKSINFACET"])
                # end_line = int(dct["END%"] * dct["TICKSINFACET"]) 
                # with m.If((tickcounter >= start_line) & (tickcounter <= end_line) & (self.synchronized == 1)):
                #     m.d.sync += lasers.eq(int("11", 2))
                # -1 as you count till range-1 in python
                # -2 as you need 1 tick to process
                with m.If(
                    tickcounter
                    >= round(dct["TICKSINFACET"] - dct["JITTERTICKS"] - 2)
                ):
                    m.d.sync += lasers.eq(int("10", 2))
                    m.next = "WAIT_STABLE"
                with m.Else():
                    m.d.sync += lasers.eq(int("00", 2))
                # if user disables synhcronization exit
                with m.If(~self.synchronize):
                    m.next = "STOP"
        if self.platform.name == "Test":
            self.laserfsm = laserfsm
        return m


class DiodeSimulator(Laserhead):
    """Wraps laser head with object to simulate photodiode

    This is purely used for testing. Photodiode is only created
    if prism motor is enabled and the laser is on so the diode
    can be triggered.
    """

    def __init__(self, platform, laser_var=None, addfifo=True):
        if laser_var is not None:
            platform.laser_var = laser_var
        super().__init__(platform)
        self.addfifo = addfifo
        self.laser0in = Signal()
        self.laser1in = Signal()
        self.enable_prism_in = Signal()
        if addfifo:
            self.write_en = Signal()
            self.write_commit = Signal()
            self.write_data = Signal(MEMWIDTH)
            self.read_en_2 = Signal()
            self.read_commit_2 = Signal()
            self.read_data_2 = Signal(MEMWIDTH)

    def elaborate(self, platform):
        if self.platform is not None:
            platform = self.platform
        m = super().elaborate(platform)

        dct = self.dct
        diodecounter = Signal(range(dct["TICKSINFACET"]))
        self.diodecounter = diodecounter

        if self.addfifo:
            m.d.comb += [
                self.enable_prism_in.eq(self.enable_prism),
                self.laser0in.eq(self.lasers[0]),
                self.laser1in.eq(self.lasers[1]),
            ]
            # fifo 1:
            fifo = TransactionalizedFIFO(
                width=MEMWIDTH, depth=platform.memdepth
            )
            m.submodules.fifo = fifo
            self.fifo = fifo
            m.d.comb += [
                fifo.write_data.eq(self.write_data),
                fifo.write_commit.eq(self.write_commit),
                fifo.write_en.eq(self.write_en),
                fifo.read_commit.eq(self.read_commit),
                fifo.read_en.eq(self.read_en),
                self.empty.eq(fifo.empty),
                fifo.read_discard.eq(self.read_discard),
                self.read_data.eq(fifo.read_data),
            ]
            # fifo 2:
            fifo2 = TransactionalizedFIFO(
                width=MEMWIDTH, depth=platform.memdepth
            )
            m.submodules.fifo2 = fifo2
            self.fifo2 = fifo2
            m.d.comb += [
                fifo2.write_data.eq(self.write_data_2),
                fifo2.write_commit.eq(self.write_commit_2),
                fifo2.write_en.eq(self.write_en_2),
                fifo2.read_commit.eq(self.read_commit_2),
                fifo2.read_en.eq(self.read_en_2),
                self.full.eq(fifo2.full),
                fifo2.write_discard.eq(self.write_discard_2),
                self.read_data_2.eq(fifo2.read_data),
            ]

        with m.If(diodecounter == (dct["TICKSINFACET"] - 1)):
            m.d.sync += diodecounter.eq(0)
        with m.Elif(diodecounter > (dct["TICKSINFACET"] - 4)):
            m.d.sync += [
                self.photodiode.eq(
                    ~(self.enable_prism_in & (self.laser0in | self.laser1in))
                ),
                diodecounter.eq(diodecounter + 1),
            ]
        with m.Else():
            m.d.sync += [
                diodecounter.eq(diodecounter + 1),
                self.photodiode.eq(1),
            ]
        self.diodecounter = diodecounter
        return m


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
        lines = [[randint(0, 1) for _ in range(dut.dct["BITSINSCANLINE"])] for _ in range(numblines)]
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
    laser_var = deepcopy(platform.laser_var)
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
    laser_var = deepcopy(platform.laser_var)
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

        for facet in range(self.platform.laser_var["FACETS"] - 1):
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

    async def checkmove(self, sim, direction, stepsperline=1, numblines=3, appendstop=True):
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

    dct = deepcopy(platform.laser_var)
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
    dct = deepcopy(platform.laser_var)
    dct["TICKSINFACET"] = 500
    dct["LASERTICKS"] = 3
    dct["SINGLE_LINE"] = False
    dct["BITSINSCANLINE"] = MEMWIDTH + 1
    FRAGMENT_ARGUMENTS = {"platform": platform, "laser_var": dct}


if __name__ == "__main__":
    unittest.main()

# NOTE: new class is created to reset settings
#       couldn't avoid this easily so kept for now
#
#       syncing behaviour is not really tested, in reality
#       prism spins up and systems goes into sync
#
#       spin up of prism is done without profile