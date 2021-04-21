import unittest

from nmigen import Signal, Elaboratable
from nmigen import Module
from luna.gateware.test import LunaGatewareTestCase, sync_test_case

from FPGAG.constants import (MEMWIDTH, INSTRUCTIONS)
from FPGAG.platforms import TestPlatform


class Laserhead(Elaboratable):
    """ Controller of laser scanner with rotating mirror or prism

        I/O signals:
        O: synchronized   -- if true, laser is in sync and prism is rotating
        O: error          -- error signal
        O: lasers         -- laser pin
        O: pwm            -- pulse for scanner motor
        O: enable_prism   -- enable pin scanner motor
        I: synchronize    -- activate synchorinzation
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
        self.read_data = Signal()
        self.empty = Signal()

    def elaborate(self, platform):
        m = Module()
        if self.platform is not None:
            platform = self.platform
        var = platform.laser_var
        var['POLY_HZ'] = var['RPM']/60
        if self.platform.name == 'Test':
            ticksinfacet = var['TICKSINFACET']
            laserticks = var['LASERTICKS']
            self.ticksinfacet = ticksinfacet
            var['CRYSTAL_HZ'] = round(ticksinfacet*var['FACETS']
                                      * var['POLY_HZ'])
            var['LASER_HZ'] = var['CRYSTAL_HZ']/laserticks
            var['SPINUP_TIME'] = 10/var['CRYSTAL_HZ']
            # TODO: stop scanline seems to affect the stable thresh?!
            # can be 30 without stopline (this is from old repo)
            var['STABLE_TIME'] = 50/var['CRYSTAL_HZ']
            var['START%'] = 2/ticksinfacet
            scanbits = 2
            var['END%'] = (laserticks*scanbits)/ticksinfacet + var['START%']
            self.var = var
        # parameter creation
        ticksinfacet = round(var['CRYSTAL_HZ']/(var['POLY_HZ']*var['FACETS']))
        laserticks = int(var['CRYSTAL_HZ']/var['LASER_HZ'])
        jitterticks = round(0.5*laserticks)
        if var['END%'] > round(1-(jitterticks+1)
                               / ticksinfacet):
            raise Exception("Invalid settings, END% too high")
        bitsinscanline = round((ticksinfacet*(var['END%']-var['START%']))
                               / laserticks)
        if bitsinscanline <= 0:
            raise Exception("Bits in scanline invalid")
        # Pulse generator for prism motor
        polyperiod = int(var['CRYSTAL_HZ']/(var['POLY_HZ']*6*2))
        pwmcnt = Signal(range(polyperiod))
        # photodiode_triggered
        photodiodecnt = Signal(range(ticksinfacet*2))
        triggered = Signal()
        with m.If(photodiodecnt < (ticksinfacet*2-1)):
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
                         pwmcnt.eq(polyperiod-1)]
        with m.Else():
            m.d.sync += pwmcnt.eq(pwmcnt-1)
        # Laser FSM
        facetcnt = Signal(range(var['FACETS']))
        spinupticks = round(var['SPINUP_TIME']*var['CRYSTAL_HZ'])
        stableticks = round(var['STABLE_TIME']*var['CRYSTAL_HZ'])
        stablecntr = Signal(range(max(spinupticks, stableticks)))
        stablethresh = Signal(range(stableticks))
        lasercnt = Signal(range(laserticks))
        scanbit = Signal(range(bitsinscanline+1))
        tickcounter = Signal(range(ticksinfacet*2))
        photodiode = self.photodiode
        read_data = self.read_data
        read_old = Signal.like(read_data)
        readbit = Signal(range(MEMWIDTH))
        photodiode_d = Signal()
        lasers = self.lasers
        if self.platform.name == 'Test':
            self.tickcounter = tickcounter
            self.ticksinfacet = ticksinfacet
            self.jitterticks = jitterticks

        with m.FSM(reset='RESET') as laserfsm:
            with m.State('RESET'):
                m.d.sync += self.error.eq(0)
                m.next = 'STOP'
            with m.State('STOP'):
                m.d.sync += [stablethresh.eq(stableticks-1),
                             stablecntr.eq(0),
                             self.enable_prism.eq(1),
                             readbit.eq(0),
                             facetcnt.eq(0),
                             scanbit.eq(0),
                             lasercnt.eq(0),
                             lasers.eq(0)]
                with m.If(self.synchronize):
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
                with m.If(stablecntr > spinupticks-1):
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
                    with m.If((tickcounter > ticksinfacet-jitterticks) &
                              (tickcounter < ticksinfacet+jitterticks)):
                        with m.If(facetcnt == var['FACETS']-1):
                            m.d.sync += facetcnt.eq(0)
                        with m.Else():
                            m.d.sync += [facetcnt.eq(facetcnt+1),
                                         stablecntr.eq(0)]
                        with m.If(var['SINGLE_FACET'] & (facetcnt > 0)):
                            m.next = 'WAIT_END'
                        with m.Else():
                            # TODO: 10 is too high, should be lower
                            thresh = min(round(10.1*ticksinfacet), stableticks)
                            m.d.sync += stablethresh.eq(thresh)
                            m.next = 'READ_INSTRUCTION'
                    with m.Else():
                        m.next = 'WAIT_END'
                with m.Else():
                    m.d.sync += tickcounter.eq(tickcounter+1)
            with m.State('READ_INSTRUCTION'):
                m.d.sync += tickcounter.eq(tickcounter+1)
                with m.If(self.empty):
                    m.next = 'WAIT_END'
                with m.Else():
                    m.d.sync += self.read_en.eq(1)
                    m.next = 'PARSE_HEAD'
            with m.State('PARSE_HEAD'):
                m.d.sync += [self.read_en.eq(0), tickcounter.eq(tickcounter+1)]
                # with m.If(read_data == INSTRUCTIONS.STOP):
                #     m.next = 'STOP'
                with m.If(read_data == INSTRUCTIONS.SCANLINE):
                    m.next = 'WAIT_FOR_DATA_RUN'
                with m.Else():
                    m.d.sync += self.error.eq(1)
                    m.next = 'STOP'
            with m.State('WAIT_FOR_DATA_RUN'):
                m.d.sync += [tickcounter.eq(tickcounter+1),
                             readbit.eq(0),
                             scanbit.eq(0),
                             lasercnt.eq(0)]
                tickcnt_thresh = int(var['START%']*ticksinfacet-2)
                with m.If(tickcounter >= tickcnt_thresh):
                    m.next = 'DATA_RUN'
            with m.State('DATA_RUN'):
                m.d.sync += tickcounter.eq(tickcounter+1)
                # NOTE:
                #      readbit is your current position in memory
                #      scanbit current byte position in scanline
                #      lasercnt used to pulse laser at certain freq
                with m.If(lasercnt == 0):
                    with m.If(scanbit >= bitsinscanline):
                        m.next = 'WAIT_END'
                    with m.Else():
                        m.d.sync += [lasercnt.eq(laserticks-1),
                                     scanbit.eq(scanbit+1),
                                     self.lasers[0].eq(read_old[0])]
                # read from memory before the spinup
                # it is triggered here again, so fresh data is available
                # once the end is reached
                # if read bit is 0, trigger a read out unless the next byte
                # is outside of line
                        with m.If(readbit == 0):
                            # TODO: what if fifo is empty!?
                            m.d.sync += [self.read_en.eq(1),
                                         readbit.eq(readbit+1),
                                         read_old.eq(read_old >> 1)]
                        # final read bit copy memory
                        # move to next address, i.e. byte, if end is reached
                        with m.Elif(readbit == MEMWIDTH-1):
                            m.d.sync += read_old.eq(read_data)
                        with m.Else():
                            m.d.sync += [readbit.eq(readbit+1),
                                         read_old.eq(read_old >> 1)]
                with m.Else():
                    m.d.sync += lasercnt.eq(lasercnt-1)
            with m.State('WAIT_END'):
                m.d.sync += [stablecntr.eq(stablecntr+1),
                             tickcounter.eq(tickcounter+1)]
                with m.If(tickcounter >= round(ticksinfacet-jitterticks-1)):
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
    def elaborate(self, platform):
        if self.platform is not None:
            platform = self.platform
        m = super().elaborate(platform)
        var = platform.laser_var
        ticksinfacet = var['TICKSINFACET']
        diodecounter = Signal(range(ticksinfacet))
        self.diodecounter = diodecounter

        with m.If(diodecounter == (ticksinfacet-1)):
            m.d.sync += diodecounter.eq(0)
        with m.Elif(diodecounter > (ticksinfacet-4)):
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
        var = dut.var
        while (yield dut.pwm) == 0:
            yield
        cnt = 0
        while (yield dut.pwm) == 1:
            cnt += 1
            yield
        self.assertEqual(cnt,
                         int(var['CRYSTAL_HZ']/(var['POLY_HZ']*6*2))
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

    @sync_test_case
    def test_sync(self):
        '''photodiode should be triggered state
           read instruction is reached
        '''
        dut = self.dut
        yield dut.synchronize.eq(1)
        yield from self.waituntilState('SPINUP')
        self.assertEqual((yield dut.error), 0)
        yield from self.waituntilState('WAIT_STABLE')
        yield from self.waituntilState('READ_INSTRUCTION')


if __name__ == "__main__":
    unittest.main()

# which test do I need
#  ' remains in stable without write'
#  ' invalid line resuls in error'
#  ' stop scanline end exposure'
#  ' scanline repeated'
#  ' scanline repeated single facet'
#  ' scanline ringbuffer'
