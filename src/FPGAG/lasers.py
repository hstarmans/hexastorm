from nmigen import Signal, Elaboratable
from nmigen import Module

from FPGAG.constants import (MEMWIDTH, INSTRUCTIONS)


class Laserhead(Elaboratable):
    """ Controller of laser scanner with rotating mirror or prism

        I/O signals:
        O: synchronized   -- if true, laser is in sync and prism is rotating
        O: error          -- error signal
        O: lasers         -- laser pin
        O: pwm            -- pulse for scanner motor
        O: enablepin      -- enable pin scanner motor
        I: synchronize    -- activate synchorinzation
        I: photodiode     -- trigger for photodiode
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
        self.divider = divider
        self.status = Signal()
        self.lasers = Signal(2)
        self.pwm = Signal()
        self.enablepin = Signal()
        self.synchronize = Signal()
        self.error = Signal()
        self.photodiode = Signal()
        self.read_commit = Signal()
        self.read_en = Signal()
        self.read_data = Signal()
        self.empty = Signal()

    def elaborate(self, platform):
        m = Module()
        if self.platform is not None:
            platform = self.platform
        var = platform.laserhead
        # parameter creation
        ticksinfacet = round(var['CRYSTAL_HZ']/(var['RPM']/60*var['FACETS']))
        laserticks = int(var['CRYSTAL_HZ']/var['LASER_HZ'])
        jitterticks = round(0.5*laserticks)
        if var['END%'] > round(1-(self.JITTERTICKS+1)
                               / self.ticksinfacet):
            raise Exception("Invalid settings, END% too high")
        bitsinscanline = round((self.ticksinfacet*(var['END%']-var['START%']))
                               / laserticks)
        if bitsinscanline <= 0:
            raise Exception("Bits in scanline invalid")
        # Pulse generator for prism motor
        polyperiod = int(var['CRYSTAL_HZ']/(var['RPM']/60)/(6*2))
        pwmcnt = Signal(range(polyperiod))
        poly_en = Signal()
        # pwm is always created but can be deactivated
        with m.If(pwmcnt == 0):
            m.d.sync += [self.pwm.eq(~self.pwm),
                         pwmcnt.eq(polyperiod-1)]
        with m.Else():
            m.d.sync += pwmcnt.eq(pwmcnt+1)
        # Laser FSM
        facetcnt = Signal(max=var['FACETS'])
        spinupticks = round(var['SPINUP_TIME']*var['CRYSTAL_HZ'])
        stableticks = round(var['STABLE_TIME']*var['CRYSTAL_HZ'])
        stablecntr = Signal(max=max(spinupticks, stableticks))
        stablethresh = Signal(range(stableticks))
        lasercnt = Signal(range(laserticks))
        scanbit = Signal(range(bitsinscanline+1))
        tickcounter = Signal(range(self.ticksinfacet*2))
        photodiode = self.photodiode
        read_data = self.read_data
        read_old = Signal.like(read_data)
        readbit = Signal(range(MEMWIDTH))
        photodiode_d = Signal()
        lasers = self.lasers
        with m.FSM(reset='RESET', name='laserfsm'):
            with m.State('RESET'):
                m.d.sync += self.error.eq(0)
                m.next = 'STOP'
            with m.State('STOP'):
                m.d.sync += [stablethresh.eq(stableticks-1),
                             stablecntr.eq(0),
                             poly_en.eq(0),
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
                                     poly_en.eq(1)]
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
                with m.If(stablecntr > stablethresh):
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
                with m.If(self.emtpy):
                    m.next = 'WAIT_END'
                with m.Else():
                    m.d.sync += self.read_en.eq(1)
                    m.next = 'PARSE_HEAD'
            with m.State('PARSE_HEAD'):
                m.d.sync += [self.read_en.eq(0), tickcounter.eq(tickcounter+1)]
                with m.If(read_data == INSTRUCTIONS.STOP):
                    m.next = 'STOP'
                with m.Elif(read_data == INSTRUCTIONS.SCAN):
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
                with m.If(self.tickcounter >= tickcnt_thresh):
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
                    m.next = 'STATE_WAIT_STABLE'
