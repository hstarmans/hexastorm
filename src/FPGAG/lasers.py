import unittest
from struct import unpack

from nmigen import Signal, Elaboratable
from nmigen import Module
from luna.gateware.test import LunaGatewareTestCase, sync_test_case
from luna.gateware.memory import TransactionalizedFIFO

import FPGAG.controller as controller
from FPGAG.constants import (MEMWIDTH, WORD_BYTES, INSTRUCTIONS)
from FPGAG.platforms import TestPlatform


def params(platform):
    '''determines paramateres for laser scanner

    returns dictionary
    '''
    var = platform.laser_var
    var['POLY_HZ'] = var['RPM']/60
    if platform.name == 'Test':
        var['CRYSTAL_HZ'] = round(var['TICKSINFACET']*var['FACETS']
                                  * var['POLY_HZ'])
        var['LASER_HZ'] = var['CRYSTAL_HZ']/var['LASERTICKS']
        var['SPINUP_TIME'] = 10/var['CRYSTAL_HZ']
        # TODO: stop scanline seems to affect the stable thresh?!
        # can be 30 without stopline (this is from old repo)
        var['STABLE_TIME'] = 50/var['CRYSTAL_HZ']
        var['START%'] = 2/var['TICKSINFACET']
        scanbits = 2
        var['END%'] = ((var['LASERTICKS']*scanbits)/var['TICKSINFACET']
                       + var['START%'])
    # parameter creation
    var['ticksinfacet'] = round(var['CRYSTAL_HZ']/(var['POLY_HZ']
                                * var['FACETS']))
    var['LASERTICKS'] = int(var['CRYSTAL_HZ']/var['LASER_HZ'])
    var['jitterticks'] = round(0.5*var['LASERTICKS'])
    if var['END%'] > round(1-(var['jitterticks']+1)
                           / var['ticksinfacet']):
        raise Exception("Invalid settings, END% too high")
    var['bitsinscanline'] = round((var['ticksinfacet'] *
                                  (var['END%']-var['START%']))
                                  / var['LASERTICKS'])
    if var['bitsinscanline'] <= 0:
        raise Exception("Bits in scanline invalid")
    var['spinupticks'] = round(var['SPINUP_TIME']*var['CRYSTAL_HZ'])
    var['stableticks'] = round(var['STABLE_TIME']*var['CRYSTAL_HZ'])
    var['polyperiod'] = int(var['CRYSTAL_HZ']/(var['POLY_HZ']*6*2))
    return var


class Laserhead(Elaboratable):
    """ Controller of laser scanner with rotating mirror or prism

        I/O signals:
        O: synchronized   -- if true, laser is in sync and prism is rotating
        I: synchronize    -- activate synchronization
        I: exopose_start  -- start reading lines and exposing
        O: exopose_finish -- exposure is finished
        O: error          -- error signal
        O: lasers         -- laser pin
        O: pwm            -- pulse for scanner motor
        O: enable_prism   -- enable pin scanner motor
        I: photodiode     -- trigger for photodiode
        O: photodiode_t   -- high if photodiode triggered in this cycle
        O: read_commit    -- finalize read transactionalizedfifo
        O: read_en        -- enable read transactionalizedfifo
        I: read_data      -- read data from transactionalizedfifo
        I: empty          -- signal wether fifo is empty
    """
    def __init__(self, platform=None, divider=50, top=False):
        '''
        top        -- trigger synthesis of module
        platform   -- pass test platform
        divider    -- original clock of 100 MHz via PLL reduced to 50 MHz
                      if this is divided by 50 laser state updated
                      with 1 MHz
        '''
        self.platform = platform
        self.divider = divider
        self.status = Signal()
        self.lasers = Signal(2)
        self.pwm = Signal()
        self.enable_prism = Signal()
        self.synchronize = Signal()
        self.error = Signal()
        self.photodiode = Signal()
        self.photodiode_t = Signal()
        self.read_commit = Signal()
        self.read_en = Signal()
        self.read_data = Signal(MEMWIDTH)
        self.empty = Signal()
        self.expose_finished = Signal()
        self.expose_start = Signal()

    def elaborate(self, platform):
        m = Module()
        if self.platform is not None:
            platform = self.platform

        dct = params(platform)

        # Pulse generator for prism motor
        pwmcnt = Signal(range(dct['polyperiod']))
        # photodiode_triggered
        photodiodecnt = Signal(range(dct['ticksinfacet']*2))
        triggered = Signal()
        with m.If(photodiodecnt < (dct['ticksinfacet']*2-1)):
            with m.If(self.photodiode):
                m.d.sync += triggered.eq(1)
            m.d.sync += photodiodecnt.eq(photodiodecnt+1)
        with m.Else():
            m.d.sync += [self.photodiode_t.eq(triggered),
                         photodiodecnt.eq(0),
                         triggered.eq(0)]
        # pwm is always created but can be deactivated
        with m.If(pwmcnt == 0):
            m.d.sync += [self.pwm.eq(~self.pwm),
                         pwmcnt.eq(dct['polyperiod']-1)]
        with m.Else():
            m.d.sync += pwmcnt.eq(pwmcnt-1)
        # Laser FSM
        facetcnt = Signal(range(dct['FACETS']))

        stablecntr = Signal(range(max(dct['spinupticks'], dct['stableticks'])))
        stablethresh = Signal(range(dct['stableticks']))
        lasercnt = Signal(range(dct['LASERTICKS']))
        scanbit = Signal(range(dct['bitsinscanline']+1))
        tickcounter = Signal(range(dct['ticksinfacet']*2))
        photodiode = self.photodiode
        read_data = self.read_data
        read_old = Signal.like(read_data)
        readbit = Signal(range(MEMWIDTH))
        photodiode_d = Signal()
        lasers = self.lasers
        if self.platform.name == 'Test':
            self.tickcounter = tickcounter
            self.dct = dct
            # needed for line verify
            self.scanbit = scanbit
            self.lasercnt = lasercnt

        process_lines = Signal()
        expose_start_d = Signal()
        m.d.sync += expose_start_d.eq(self.expose_start)
        with m.If((expose_start_d == 0) & self.expose_start):
            m.d.sync += process_lines.eq(1)

        with m.FSM(reset='RESET') as laserfsm:
            with m.State('RESET'):
                m.d.sync += self.error.eq(0)
                m.next = 'STOP'
            with m.State('STOP'):
                m.d.sync += [stablethresh.eq(dct['stableticks']-1),
                             stablecntr.eq(0),
                             self.enable_prism.eq(1),
                             readbit.eq(0),
                             facetcnt.eq(0),
                             scanbit.eq(0),
                             lasercnt.eq(0),
                             lasers.eq(0)]
                with m.If(self.synchronize | process_lines):
                    # laser is off, photodiode cannot be triggered
                    # TODO: add check that fifo is not empty
                    with m.If(self.photodiode == 0):
                        m.d.sync += self.error.eq(1)
                        m.next = 'STOP'
                    with m.Else():
                        m.d.sync += [self.error.eq(0),
                                     self.enable_prism.eq(0)]
                        m.next = 'SPINUP'
            with m.State('SPINUP'):
                with m.If(stablecntr > dct['spinupticks']-1):
                    # turn on laser
                    m.d.sync += [self.lasers.eq(int('1'*2, 2)),
                                 stablecntr.eq(0)]
                    m.next = 'WAIT_STABLE'
                with m.Else():
                    m.d.sync += stablecntr.eq(stablecntr+1)
            with m.State('WAIT_STABLE'):
                m.d.sync += [stablecntr.eq(stablecntr+1),
                             photodiode_d.eq(photodiode)]
                with m.If(stablecntr >= stablethresh):
                    m.d.sync += self.error.eq(1)
                    m.next = 'STOP'
                with m.Elif(~photodiode & ~photodiode_d):
                    m.d.sync += [tickcounter.eq(0),
                                 lasers.eq(0)]
                    with m.If((tickcounter > dct['ticksinfacet']
                              - dct['jitterticks']) &
                              (tickcounter < dct['ticksinfacet']
                              + dct['jitterticks'])):
                        with m.If(facetcnt == dct['FACETS']-1):
                            m.d.sync += facetcnt.eq(0)
                        with m.Else():
                            m.d.sync += [facetcnt.eq(facetcnt+1),
                                         stablecntr.eq(0)]
                        with m.If(dct['SINGLE_FACET'] & (facetcnt > 0)):
                            m.next = 'WAIT_END'
                        with m.Elif(self.empty | ~process_lines):
                            m.next = 'WAIT_END'
                        with m.Else():
                            # TODO: 10 is too high, should be lower
                            thresh = min(round(10.1*dct['ticksinfacet']),
                                         dct['stableticks'])
                            m.d.sync += [stablethresh.eq(thresh),
                                         self.read_en.eq(1)]
                            m.next = 'READ_INSTRUCTION'
                    with m.Else():
                        m.next = 'WAIT_END'
                with m.Else():
                    m.d.sync += tickcounter.eq(tickcounter+1)
            with m.State('READ_INSTRUCTION'):
                m.d.sync += [self.read_en.eq(0), tickcounter.eq(tickcounter+1)]
                with m.If(read_data == INSTRUCTIONS.SCANLINE):
                    m.next = 'WAIT_FOR_DATA_RUN'
                with m.Elif(read_data == INSTRUCTIONS.LASTSCANLINE):
                    m.d.sync += [self.expose_finished.eq(1),
                                 process_lines.eq(0)]
                with m.Else():
                    m.d.sync += self.error.eq(1)
                    m.next = 'STOP'
            with m.State('WAIT_FOR_DATA_RUN'):
                m.d.sync += [tickcounter.eq(tickcounter+1),
                             readbit.eq(0),
                             scanbit.eq(0),
                             lasercnt.eq(0)]
                tickcnt_thresh = int(dct['START%']*dct['ticksinfacet'])
                assert tickcnt_thresh > 0
                with m.If(tickcounter >= tickcnt_thresh):
                    m.d.sync += self.read_en.eq(1)
                    m.next = 'DATA_RUN'
            with m.State('DATA_RUN'):
                m.d.sync += tickcounter.eq(tickcounter+1)
                # NOTE:
                #      readbit is your current position in memory
                #      scanbit current byte position in scanline
                #      lasercnt used to pulse laser at certain freq
                with m.If(lasercnt == 0):
                    with m.If(scanbit >= dct['bitsinscanline']):
                        m.next = 'WAIT_END'
                    with m.Else():
                        m.d.sync += [lasercnt.eq(dct['LASERTICKS']-1),
                                     scanbit.eq(scanbit+1)]
                        with m.If(readbit == 0):
                            m.d.sync += self.lasers[0].eq(self.read_data[0])
                        with m.Else():
                            m.d.sync += self.lasers[0].eq(read_old[0])

                        with m.If(readbit == 0):
                            m.d.sync += [self.read_en.eq(0),
                                         read_old.eq(self.read_data >> 1),
                                         readbit.eq(readbit+1)]
                        # final read bit copy memory
                        # move to next address, i.e. byte, if end is reached
                        with m.Elif(readbit == MEMWIDTH-1):
                            # If fifo is empty it will give errors later
                            # so it can be ignored here
                            # Only grab a new line if more than current
                            # is needed
                            with m.If(scanbit >= (dct['bitsinscanline']-2)):
                                m.d.sync += self.read_en.eq(1)
                            m.d.sync += readbit.eq(0)
                        with m.Else():
                            m.d.sync += [readbit.eq(readbit+1),
                                         read_old.eq(read_old >> 1)]
                with m.Else():
                    m.d.sync += lasercnt.eq(lasercnt-1)
            with m.State('WAIT_END'):
                m.d.sync += [stablecntr.eq(stablecntr+1),
                             tickcounter.eq(tickcounter+1)]
                with m.If(tickcounter >= round(dct['ticksinfacet']
                          - dct['jitterticks']-1)):
                    m.d.sync += lasers.eq(int('11', 2))
                    m.next = 'WAIT_STABLE'
        if self.platform.name == 'Test':
            self.laserfsm = laserfsm
        return m


class DiodeSimulator(Laserhead):
    """ Wraps laser head with object which simulates photodiode

        This is purely used for testing. Photodiode is only created
        if prism motor is enabled and the laser is on so the diode
        can be triggered.
    """
    def __init__(self, platform=None, divider=50, top=False):
        super().__init__(platform, divider, top)
        self.write_en = Signal()
        self.write_commit = Signal()
        self.write_data = Signal(MEMWIDTH)

    def elaborate(self, platform):
        if self.platform is not None:
            platform = self.platform
        m = super().elaborate(platform)
        dct = params(platform)
        diodecounter = Signal(range(dct['ticksinfacet']))
        self.diodecounter = diodecounter

        fifo = TransactionalizedFIFO(width=MEMWIDTH,
                                     depth=platform.memdepth)
        m.submodules.fifo = fifo
        self.fifo = fifo
        m.d.comb += [fifo.write_data.eq(self.write_data),
                     fifo.write_commit.eq(self.write_commit),
                     fifo.write_en.eq(self.write_en),
                     fifo.read_commit.eq(self.read_commit),
                     fifo.read_en.eq(self.read_en),
                     self.empty.eq(fifo.empty),
                     self.read_data.eq(fifo.read_data)]

        with m.If(diodecounter == (dct['ticksinfacet']-1)):
            m.d.sync += diodecounter.eq(0)
        with m.Elif(diodecounter > (dct['ticksinfacet']-4)):
            m.d.sync += [self.photodiode.eq(~((self.enable_prism == 0)
                                            & (self.lasers != 0))),
                         diodecounter.eq(diodecounter+1)]
        with m.Else():
            m.d.sync += [diodecounter.eq(diodecounter+1),
                         self.photodiode.eq(1)]
        return m


class LaserheadTest(LunaGatewareTestCase):
    'Test laserhead without triggering photodiode'
    platform = TestPlatform()
    FRAGMENT_UNDER_TEST = Laserhead
    FRAGMENT_ARGUMENTS = {'platform': platform, 'divider': 1}

    def initialize_signals(self):
        '''If not triggered the photodiode is high'''
        yield self.dut.photodiode.eq(1)

    def getState(self, fsm=None):
        if fsm is None:
            fsm = self.dut.laserfsm
        return fsm.decoding[(yield fsm.state)]

    def waituntilState(self, state, fsm=None):
        while (yield from self.getState(fsm)) != state:
            yield

    def assertState(self, state, fsm=None):
        self.assertEqual(self.getState(state), state)

    @sync_test_case
    def test_pwmpulse(self):
        '''pwm pulse generation test'''
        dut = self.dut
        dct = params(self.platform)
        while (yield dut.pwm) == 0:
            yield
        cnt = 0
        while (yield dut.pwm) == 1:
            cnt += 1
            yield
        self.assertEqual(cnt,
                         int(dct['CRYSTAL_HZ']/(dct['POLY_HZ']*6*2))
                         )

    @sync_test_case
    def test_sync(self):
        '''error is raised if laser not synchronized'''
        dut = self.dut
        yield dut.synchronize.eq(1)
        yield from self.waituntilState('SPINUP')
        self.assertEqual((yield dut.error), 0)
        yield from self.waituntilState('WAIT_STABLE')
        yield from self.waituntilState('STOP')
        self.assertEqual((yield dut.error), 1)


class DiodeTest(LaserheadTest):
    'Test laserhead while triggering photodiode'
    platform = TestPlatform()
    FRAGMENT_UNDER_TEST = DiodeSimulator
    FRAGMENT_ARGUMENTS = {'platform': platform, 'divider': 1}

    def initialize_signals(self):
        yield from super().initialize_signals()
        self.host = controller.Host(self.platform)

    def checkline(self, bitlst):
        'it is verified wether the laser produces the pattern in bitlist'
        dut = self.dut
        self.assertEqual((yield dut.empty), False)
        yield from self.waituntilState('READ_INSTRUCTION')
        yield
        if len(bitlst) == 0:
            self.assertEqual((yield dut.error), False)
            self.assertEqual((yield dut.expose_finished), True)
        else:
            yield from self.waituntilState('DATA_RUN')
            yield
            for idx, bit in enumerate(bitlst):
                assert (yield dut.lasercnt) == dut.dct['LASERTICKS']-1
                assert (yield dut.scanbit) == idx+1
                for _ in range(dut.dct['LASERTICKS']):
                    assert (yield dut.lasers[0]) == bit
                    yield
        yield from self.waituntilState('WAIT_END')
        self.assertEqual((yield self.dut.error), False)

    def write_line(self, bitlist):
        '''write line to fifo

        This is a helper function to allow testing of the module
        without dispatcher and parser
        '''
        bytelst = self.host.bittobytelist(bitlist)
        dut = self.dut
        for i in range(0, len(bytelst), WORD_BYTES):
            lst = bytelst[i:i+WORD_BYTES]
            number = unpack('Q', bytearray(lst))[0]
            yield dut.write_data.eq(number)
            yield from self.pulse(dut.write_en)
        yield from self.pulse(dut.write_commit)

    @sync_test_case
    def test_sync(self):
        '''photodiode should be triggered state
           wait end is reached
        '''
        dut = self.dut
        yield dut.synchronize.eq(1)
        yield from self.waituntilState('SPINUP')
        self.assertEqual((yield dut.error), 0)
        for _ in range(3):
            yield from self.waituntilState('WAIT_STABLE')
            yield from self.waituntilState('WAIT_END')

    @sync_test_case
    def test_stopline(self):
        'verify data run is not reached when stopline is sent'
        line = []
        dut = self.dut
        yield from self.write_line(line)
        yield dut.synchronize.eq(1)
        yield from self.pulse(dut.expose_start)
        self.assertEqual((yield dut.empty), 0)
        yield from self.waituntilState('SPINUP')
        yield from self.waituntilState('WAIT_STABLE')
        yield from self.checkline(line)
        yield from self.waituntilState('WAIT_END')
        yield from self.waituntilState('WAIT_STABLE')

    @sync_test_case
    def test_scanlineringbuffer(self):
        'several scanline'
        dut = self.dut
        lines = [[1, 1], [1, 1], [0, 1], []]
        for line in lines:
            yield from self.write_line(line)
        yield dut.synchronize.eq(1)
        yield from self.pulse(dut.expose_start)
        for line in lines:
            yield from self.checkline(line)


if __name__ == "__main__":
    unittest.main()

# which test do I need
#  ' remains in stable without write'
#  ' invalid line resuls in error'
#  ' stop scanline end exposure'
#  ' scanline repeated'
#  ' scanline repeated single facet'
#  ' scanline ringbuffer'
