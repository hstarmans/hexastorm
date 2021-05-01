import unittest
from random import randint
from copy import deepcopy

import numpy as np
from numpy.testing import assert_array_equal
from nmigen import Signal, Elaboratable, signed, Cat
from nmigen import Module
from nmigen.hdl.mem import Array
from luna.gateware.utils.cdc import synchronize
from luna.gateware.interface.spi import SPICommandInterface, SPIBus
from luna.gateware.memory import TransactionalizedFIFO
from luna.gateware.test.utils import sync_test_case
from luna.gateware.interface.spi import SPIGatewareTestCase

from FPGAG.controller import Host, Memfull
from FPGAG.movement import Polynomal
from FPGAG.platforms import TestPlatform
from FPGAG.resources import get_all_resources
from FPGAG.lasers import Laserhead
from FPGAG.constants import (COMMAND_BYTES, WORD_BYTES, STATE, INSTRUCTIONS,
                             MEMWIDTH, COMMANDS, DEGREE, FREQ)


class SPIParser(Elaboratable):
    """ Parses and replies to commands over SPI

    The following commmands are possible
      status -- send back state of the peripheriral
      start  -- enable execution of gcode
      stop   -- halt execution of gcode
      write  -- write instruction to FIFO or report memory is full

    I/O signals:
        I/O: Spibus       -- spi bus connected to peripheral
        I: positions      -- positions of stepper motors
        I: pin state      -- used to get the value of select pins at client
        I: read_commit    -- finalize read transactionalizedfifo
        I: read_en        -- enable read transactionalizedfifo
        I: dispatcherror  -- error while processing stored command from spi
        O: execute        -- start processing gcode
        O: read_data      -- read data from transactionalizedfifo
        O: empty          -- transactionalizedfifo is empty
    """
    def __init__(self, platform, top=False):
        """
        platform  -- pass test platform
        top       -- trigger synthesis of module
        """
        self.platform = platform
        self.top = top

        self.spi = SPIBus()
        self.position = Array(Signal(signed(64))
                              for _ in range(platform.motors))
        self.pinstate = Signal(8)
        self.read_commit = Signal()
        self.read_en = Signal()
        self.dispatcherror = Signal()
        self.execute = Signal()
        self.read_data = Signal(MEMWIDTH)
        self.empty = Signal()

    def elaborate(self, platform):
        m = Module()
        if platform and self.top:
            board_spi = platform.request("debug_spi")
            spi2 = synchronize(m, board_spi)
            m.d.comb += self.spi.connect(spi2)
        if self.platform:
            platform = self.platform
        spi = self.spi
        interf = SPICommandInterface(command_size=COMMAND_BYTES*8,
                                     word_size=WORD_BYTES*8)
        m.d.comb += interf.spi.connect(spi)
        m.submodules.interf = interf
        # FIFO connection
        fifo = TransactionalizedFIFO(width=MEMWIDTH,
                                     depth=platform.memdepth)
        if platform.name == 'Test':
            self.fifo = fifo
        m.submodules.fifo = fifo
        m.d.comb += [self.read_data.eq(fifo.read_data),
                     fifo.read_commit.eq(self.read_commit),
                     fifo.read_en.eq(self.read_en),
                     self.empty.eq(fifo.empty)]
        # Parser
        mtrcntr = Signal(range(platform.motors))
        wordsreceived = Signal(range(platform.wordsinmove+1))
        error = Signal()
        # Peripheral state
        state = Signal(8)
        m.d.sync += [state[STATE.PARSING].eq(self.execute),
                     state[STATE.FULL].eq(fifo.space_available <= 1),
                     state[STATE.ERROR].eq(self.dispatcherror | error)]
        # remember which word we are processing
        instruction = Signal(8)
        with m.FSM(reset='RESET', name='parser'):
            with m.State('RESET'):
                m.d.sync += [self.execute.eq(0), wordsreceived.eq(0),
                             error.eq(0)]
                m.next = 'WAIT_COMMAND'
            with m.State('WAIT_COMMAND'):
                with m.If(interf.command_ready):
                    word = Cat(state[::-1], self.pinstate[::-1])
                    with m.If(interf.command == COMMANDS.EMPTY):
                        m.next = 'WAIT_COMMAND'
                    with m.Elif(interf.command == COMMANDS.START):
                        m.next = 'WAIT_COMMAND'
                        m.d.sync += self.execute.eq(1)
                    with m.Elif(interf.command == COMMANDS.STOP):
                        m.next = 'WAIT_COMMAND'
                        m.d.sync += self.execute.eq(0)
                    with m.Elif(interf.command == COMMANDS.WRITE):
                        m.d.sync += interf.word_to_send.eq(word)
                        with m.If(state[STATE.FULL] == 0):
                            m.next = 'WAIT_WORD'
                        with m.Else():
                            m.next = 'WAIT_COMMAND'
                    with m.Elif(interf.command == COMMANDS.READ):
                        m.d.sync += interf.word_to_send.eq(word)
                        m.next = 'WAIT_COMMAND'
                    with m.Elif(interf.command == COMMANDS.POSITION):
                        # position is requested multiple times for multiple
                        # motors
                        with m.If(mtrcntr < platform.motors):
                            m.d.sync += mtrcntr.eq(mtrcntr+1)
                        with m.Else():
                            m.d.sync += mtrcntr.eq(0)
                        m.d.sync += interf.word_to_send.eq(
                                                self.position[mtrcntr])
                        m.next = 'WAIT_COMMAND'
            with m.State('WAIT_WORD'):
                with m.If(interf.word_complete):
                    byte0 = interf.word_received[:8]
                    with m.If(wordsreceived == 0):
                        with m.If((byte0>0)&(byte0<6)):
                            m.d.sync += [instruction.eq(byte0),
                                         fifo.write_en.eq(1),
                                         wordsreceived.eq(wordsreceived+1),
                                         fifo.write_data.eq(
                                             interf.word_received)]
                            m.next = 'WRITE'
                        with m.Else():
                            m.d.sync += error.eq(1)
                            m.next = 'WAIT_COMMAND'
                    with m.Else():
                        m.d.sync += [fifo.write_en.eq(1),
                                     wordsreceived.eq(wordsreceived+1),
                                     fifo.write_data.eq(interf.word_received)]
                        m.next = 'WRITE'
            with m.State('WRITE'):
                m.d.sync += fifo.write_en.eq(0)
                with m.If(((instruction == INSTRUCTIONS.MOVE) &
                          (wordsreceived >= platform.wordsinmove))
                          | (instruction == INSTRUCTIONS.WRITEPIN)
                          | (instruction == INSTRUCTIONS.LASTSCANLINE)
                          | ((instruction == INSTRUCTIONS.SCANLINE) &
                          (wordsreceived >= platform.wordsinscanline))):
                    m.d.sync += [wordsreceived.eq(0),
                                 fifo.write_commit.eq(1)]
                    m.next = 'COMMIT'
                with m.Else():
                    m.next = 'WAIT_COMMAND'
            with m.State('COMMIT'):
                m.d.sync += fifo.write_commit.eq(0)
                m.next = 'WAIT_COMMAND'
        return m


class Dispatcher(Elaboratable):
    """ Dispatches instructions to right submodule

        Instructions are buffered in SRAM. This module checks the buffer
        and dispatches the instructions to the corresponding module.
        This is the top module
    """
    def __init__(self, platform=None, divider=50):
        """
            platform  -- used to pass test platform
            divider   -- if sys clk is 50 MHz and divider is 50
                        motor state is update with 1 Mhz
        """
        self.platform = platform
        self.divider = divider

    def elaborate(self, platform):
        m = Module()
        # Parser
        parser = SPIParser(self.platform)
        m.submodules.parser = parser
        # Polynomal Move
        polynomal = Polynomal(self.platform, self.divider)
        m.submodules.polynomal = polynomal
        # Laserscan Head
        laserhead = Laserhead(self.platform, self.divider)
        m.submodules.laserhead = laserhead
        # Local laser signal clones
        enable_prism = Signal()
        lasers = Signal(2)
        # Busy signal
        busy = Signal()
        m.d.comb += busy.eq(polynomal.busy)
        # position adder
        busy_d = Signal()
        m.d.sync += busy_d.eq(polynomal.busy)
        if platform:
            board_spi = platform.request("debug_spi")
            spi = synchronize(m, board_spi)
            m.submodules.car = platform.clock_domain_generator()
            laserheadpins = platform.request("laserscanner")
            steppers = [res for res in get_all_resources(platform, "stepper")]
            assert len(steppers) != 0
        else:
            platform = self.platform
            self.spi = SPIBus()
            self.parser = parser
            self.pol = polynomal
            spi = synchronize(m, self.spi)
            self.laserheadpins = platform.laserhead
            self.laserhead = laserhead
            self.steppers = steppers = platform.steppers
            self.busy = busy
            laserheadpins = self.platform.laserhead
        coeffcnt = Signal(range(len(polynomal.coeff)))
        # connect laserhead
        m.d.comb += [
            laserheadpins.pwm.eq(laserhead.pwm),
            laserheadpins.en.eq(laserhead.enable_prism & enable_prism),
            laserhead.photodiode.eq(laserheadpins.photodiode),
            laserheadpins.laser0.eq(laserhead.lasers[0] | lasers[0]),
            laserheadpins.laser1.eq(laserhead.lasers[1] | lasers[1]),
        ]
        # connect motors
        for idx, stepper in enumerate(steppers):
            m.d.comb += [stepper.step.eq(polynomal.step[idx] &
                                         ((stepper.limit == 0) | stepper.dir)),
                         stepper.dir.eq(polynomal.dir[idx]),
                         parser.pinstate[idx].eq(stepper.limit)]
        m.d.comb += (parser.pinstate[len(steppers)+1].
                     eq(laserhead.photodiode_t))
        # connect spi
        m.d.comb += parser.spi.connect(spi)
        with m.If((busy_d == 1) & (busy == 0)):
            for idx, position in enumerate(parser.position):
                m.d.sync += position.eq(position+polynomal.totalsteps[idx])
        with m.FSM(reset='RESET', name='dispatcher'):
            with m.State('RESET'):
                m.next = 'WAIT_INSTRUCTION'
            with m.State('WAIT_INSTRUCTION'):
                m.d.sync += [parser.read_commit.eq(0), polynomal.start.eq(0)]
                with m.If((parser.empty == 0) & parser.execute & (busy == 0)):
                    m.d.sync += parser.read_en.eq(1)
                    m.next = 'PARSEHEAD'
            # check which instruction we r handling
            with m.State('PARSEHEAD'):
                byte0 = parser.read_data[:8]
                with m.If(byte0 == INSTRUCTIONS.MOVE):
                    m.d.sync += [polynomal.ticklimit.eq(parser.read_data[8:]),
                                 parser.read_en.eq(0),
                                 coeffcnt.eq(0)]
                    m.next = 'MOVE_POLYNOMAL'
                with m.Elif(byte0 == INSTRUCTIONS.WRITEPIN):
                    pins = Cat(lasers, enable_prism)
                    m.d.sync += [pins.eq(parser.read_data[8:]),
                                 parser.read_commit.eq(0)]
                    m.next = 'WAIT_INSTRUCTION'
                with m.Else():
                    m.next = 'ERROR'
                    m.d.sync += parser.dispatcherror.eq(1)
            with m.State('MOVE_POLYNOMAL'):
                with m.If(coeffcnt < len(polynomal.coeff)):
                    with m.If(parser.read_en == 0):
                        m.d.sync += parser.read_en.eq(1)
                    with m.Else():
                        m.d.sync += [polynomal.coeff[coeffcnt].eq(
                                     parser.read_data),
                                     coeffcnt.eq(coeffcnt+1),
                                     parser.read_en.eq(0)]
                with m.Else():
                    m.next = 'WAIT'
                    m.d.sync += [polynomal.start.eq(1),
                                 parser.read_commit.eq(1)]
            # NOTE: you need to wait for busy to be raised
            #       in time
            with m.State('WAIT'):
                m.next = 'WAIT_INSTRUCTION'
            # NOTE: system never recovers user must reset
            with m.State('ERROR'):
                m.next = 'ERROR'
        return m


class TestParser(SPIGatewareTestCase):
    platform = TestPlatform()
    FRAGMENT_UNDER_TEST = SPIParser
    FRAGMENT_ARGUMENTS = {'platform': platform}

    def initialize_signals(self):
        self.host = Host(self.platform)
        self.host.spi_exchange_data = self.spi_exchange_data
        yield self.dut.spi.cs.eq(0)

    def instruction_ready(self, check):
        while (yield self.dut.empty) == 1:
            yield
        # Instruction ready
        self.assertEqual((yield self.dut.empty), 0)
        self.assertEqual((yield self.dut.fifo.space_available),
                         (self.platform.memdepth - check))
        
    @sync_test_case
    def test_getposition(self):
        decimals = 3
        position = [randint(-2000, 2000) for _ in range(self.platform.motors)]
        for idx, pos in enumerate(self.dut.position):
            yield pos.eq(position[idx])
        lst = (yield from self.host.position).round(decimals)
        stepspermm = np.array(list(self.platform.stepspermm.values()))
        assert_array_equal(lst, (position/stepspermm).round(decimals))

    @sync_test_case
    def test_writescanline(self):
        yield from self.host.writeline([])
        while (yield self.dut.empty) == 1:
            yield
        yield from self.instruction_ready(self.platform.wordsinscanline)
    
    @sync_test_case
    def test_lastscanline(self):
        yield from self.host.writeline([])
        yield from self.instruction_ready(1)
        
    @sync_test_case
    def test_writepin(self):
        'write move instruction and verify FIFO is no longer empty'
        self.assertEqual((yield self.dut.empty), 1)
        yield from self.host.enable_comp(laser0=True,
                                         laser1=False,
                                         polygon=False)
        yield from self.instruction_ready(1)

    @sync_test_case
    def test_writemoveinstruction(self):
        'write move instruction and verify FIFO is no longer empty'
        self.assertEqual((yield self.dut.empty), 1)
        yield from self.host.send_move([1000],
                                       [1]*self.platform.motors,
                                       [2]*self.platform.motors,
                                       [3]*self.platform.motors)
        yield from self.instruction_ready(self.platform.wordsinmove)

    @sync_test_case
    def test_readpinstate(self):
        '''retrieve pin state'''
        def test_pins(dct):
            bitlist = dct.values()
            b = int("".join(str(i) for i in bitlist), 2)
            yield self.dut.pinstate.eq(b)
            yield
            newdct = (yield from self.host.pinstate)
            subset = ['x', 'y', 'z']
            newdct = {k: v for k, v in newdct.items() if k in subset}
            self.assertDictEqual(dct, newdct)
        yield from test_pins({'x': 0, 'y': 1, 'z': 0})
        yield from test_pins({'x': 1, 'y': 0, 'z': 1})

    @sync_test_case
    def test_enableparser(self):
        '''enables SRAM parser via command and verifies status with
        different command'''
        yield from self.host._executionsetter(True)
        self.assertEqual((yield self.dut.execute), 1)
        self.assertEqual((yield from self.host.execution), True)

    @sync_test_case
    def test_invalidwrite(self):
        '''write invalid instruction and verify error is raised'''
        command = [COMMANDS.WRITE] + [0]*WORD_BYTES
        yield from self.host.send_command(command)
        self.assertEqual((yield from self.host.error), True)

    @sync_test_case
    def test_memfull(self):
        'write move instruction until memory is full'
        platform = self.platform
        self.assertEqual((yield self.dut.empty), 1)
        # should fill the memory as move instruction is
        # larger than the memdepth
        self.assertEqual((yield from self.host.memfull()), False)
        try:
            for _ in range(platform.memdepth):
                yield from self.host.send_move([1000],
                                               [1]*platform.motors,
                                               [2]*platform.motors,
                                               [3]*platform.motors,
                                               maxtrials=1)
        except Memfull:
            pass
        self.assertEqual((yield from self.host.memfull()), True)


class TestDispatcher(SPIGatewareTestCase):
    platform = TestPlatform()
    FRAGMENT_UNDER_TEST = Dispatcher
    FRAGMENT_ARGUMENTS = {'platform': platform, 'divider': 1}

    def initialize_signals(self):
        self.host = Host(self.platform)
        self.host.spi_exchange_data = self.spi_exchange_data
        yield self.dut.spi.cs.eq(0)

    def wait_complete(self):
        '''helper method to wait for completion'''
        cntr = 0
        while (yield self.dut.busy) or (cntr < 100):
            if (yield self.dut.pol.busy):
                cntr = 0
            else:
                cntr += 1
            yield

    @sync_test_case
    def test_memfull(self):
        '''write move instruction until memory is full, execute
           and ensure there is no parser error i.e. error
        '''
        platform = self.platform
        # should fill the memory as move instruction is
        # larger than the memdepth
        self.assertEqual((yield from self.host.memfull()), False)
        try:
            for _ in range(platform.memdepth):
                yield from self.host.send_move([1000],
                                               [1]*platform.motors,
                                               [2]*platform.motors,
                                               [3]*platform.motors,
                                               maxtrials=1)
        except Memfull:
            pass
        self.assertEqual((yield from self.host.memfull()), True)
        yield from self.host._executionsetter(True)
        # data should now be processed from sram and empty become 1
        while (yield self.dut.parser.empty) == 0:
            yield
        # 2 clocks needed for error to propagate
        yield
        yield
        self.assertEqual((yield from self.host.error), False)

    @sync_test_case
    def test_readdiode(self):
        '''verify you can receive photodiode trigger

        Photodiode trigger simply checks wether the photodiode
        has been triggered for each cycle.
        '''
        yield self.dut.laserheadpins.photodiode.eq(1)
        self.assertEqual((yield self.dut.laserhead.photodiode_t),
                         False)
        val = (yield from self.host.pinstate)['photodiode_trigger']
        for _ in range(self.dut.laserhead.dct['TICKSINFACET']):
            yield
        self.assertEqual((yield self.dut.laserhead.photodiode_t),
                         True)
        self.assertEqual(val, True)

    @sync_test_case
    def test_writepin(self):
        '''verify homing procedure works correctly'''
        yield from self.host.enable_comp(laser0=True,
                                         laser1=False,
                                         polygon=False)
        # wait till instruction is received
        while (yield self.dut.parser.empty):
            yield
        yield
        # enable dispatching of code
        yield from self.host._executionsetter(True)
        # data should now be parsed and empty become 1
        while (yield self.dut.parser.empty) == 0:
            yield
        # ensure there is no error
        self.assertEqual((yield from self.host.error), False)
        self.assertEqual((yield self.dut.laserheadpins.laser0), 1)
        self.assertEqual((yield self.dut.laserheadpins.laser1), 0)
        self.assertEqual((yield self.dut.laserheadpins.en), 0)

    @sync_test_case
    def test_home(self):
        '''verify homing procedure works correctly'''
        self.host._position = np.array([0.1]*self.platform.motors)
        for i in range(self.platform.motors):
            yield self.dut.steppers[i].limit.eq(1)
        yield
        self.assertEqual((yield self.dut.parser.pinstate[0]), 1)

        yield from self.host.home_axes(axes=np.array([1]*self.platform.motors),
                                       speed=None,
                                       pos=-0.1)
        assert_array_equal(self.host._position,
                           np.array([0]*self.platform.motors))

    @sync_test_case
    def test_invalidwrite(self):
        '''write invalid instruction and verify error is raised'''
        fifo = self.dut.parser.fifo
        self.assertEqual((yield from self.host.error), False)
        # write illegal byte to queue and commit
        yield fifo.write_data.eq(0xAA)
        yield from self.pulse(fifo.write_en)
        yield from self.pulse(fifo.write_commit)
        self.assertEqual((yield self.dut.parser.empty), 0)
        # enable dispatching of code
        yield from self.host._executionsetter(True)
        # data should now be processed from sram and empty become 1
        while (yield self.dut.parser.empty) == 0:
            yield
        # 2 clocks needed for error to propagate
        yield
        yield
        self.assertEqual((yield from self.host.error), True)

    @sync_test_case
    def test_ptpmove(self, steps=[800], ticks=[30_000]):
        '''verify point to point move

        If ticks is longer than tick limit the moves is broken up.
        If the number of instruction is larger than memdepth it
        also test blocking behaviour.
        '''
        steps = steps*self.platform.motors
        mm = np.array(steps)/np.array(list(self.platform.stepspermm.values()))
        time = np.array(ticks)/FREQ
        speed = mm/time
        yield from self.host.gotopoint(mm.tolist(),
                                       speed.tolist())
        yield from self.wait_complete()
        calculated = deepcopy(self.host._position)
        assert calculated.sum() > 0
        assert_array_equal((yield from self.host.position),
                           calculated)

    @sync_test_case
    def test_movereceipt(self, ticks=10_000):
        'verify move instruction send over with send_move'
        a = list(range(1, self.platform.motors+1))
        b = list(range(3, self.platform.motors+3))
        c = list(range(5, self.platform.motors+5))
        yield from self.host.send_move([ticks],
                                       a,
                                       b,
                                       c)
        # wait till instruction is received
        while (yield self.dut.parser.empty):
            yield
        yield
        # enable dispatching of code
        yield from self.host._executionsetter(True)
        # data should now be parsed and empty become 1
        while (yield self.dut.parser.empty) == 0:
            yield
        # confirm receipt tick limit and coefficients
        self.assertEqual((yield self.dut.pol.ticklimit), 10_000)
        coefficients = [a, b, c]
        for motor in range(self.platform.motors):
            for coef in range(DEGREE):
                indx = motor*(DEGREE)+coef
                self.assertEqual((yield self.dut.pol.coeff[indx]),
                                 coefficients[coef][motor])
        while (yield self.dut.pol.busy):
            yield
        for motor in range(self.platform.motors):
            self.assertEqual((yield self.dut.pol.cntrs[motor*DEGREE]),
                             a[motor]*ticks + b[motor]*pow(ticks, 2)
                             + c[motor]*pow(ticks, 3))


if __name__ == "__main__":
    unittest.main()


# Overview:
#  the hardware consists out of the following elements
#  -- SPI command interface
#  -- transactionalized FIFO
#  -- SPI parser (basically an extension of SPI command interface)
#  -- Dispatcher --> dispatches signals to actual hardware
#  -- Polynomal integrator --> determines position via integrating counters

# TODO:
#   -- check properties like ticksinfacet propagate
#   -- in practice, position is not reached with small differences like 0.02 mm
#   -- test execution speed to ensure the right PLL is propagated
#   -- use CRC packet for tranmission failure (it is in litex but not luna)
#   -- try to replace value == 0 with ~value
#   -- xfer3 is faster in transaction
#   -- if you chip select is released parsers should return to initial state
#      now you get an error if you abort the transaction
#   -- number of ticks per motor is uniform
#   -- yosys does not give an error if you try to synthesize invalid memory
#   -- read / write commit is not perfect
#   -- simulations do not always agree with reality, around edges
