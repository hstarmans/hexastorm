import math
import sys
from collections import namedtuple

from migen.fhdl.tools import list_special_ios
from migen import *
from litex.soc.cores.clock import iCE40PLL
from litex.soc.cores.spi import SPISlave


class Scanhead(Module):
    '''
    This description can be converted to VHDL or Verilog and then converted to binary
    and uploaded to the FPGA.
    '''
    @staticmethod
    def commands():
        commands = ('STATUS', 'START', 'STOP', 'LASERTEST',
                    'MOTORTEST', 'LINETEST', 'PHOTODIODETEST',
                     'READ_D', 'WRITE_L')
        Commands = namedtuple('Commands', commands, defaults=tuple(range(len(commands))))
        return Commands()
    COMMANDS = commands.__func__()

    @staticmethod
    def states():
        states = ('STOP', 'START', 'MOTORTEST', 'LASERTEST', 'LINETEST', 'PHOTODIODETEST')
        States = namedtuple('States', states, defaults=tuple(range(len(states))))
        return States()
    STATES = states.__func__()

    @staticmethod
    def instructions():
        instructions = ('SCAN', 'STOP')
        Instructions = namedtuple('Instructions', instructions, defaults=tuple(range(1,len(instructions)+1)))
        return Instructions()
    INSTRUCTIONS = instructions.__func__()

    @staticmethod
    def errors():
        '''errors
        Multiple errors can be present at the same time. This is different from state or command
        as these can only be equal to one value.
        If all bits are zero, there is no error.
        Returned is the bit which equals the error
        '''
        errors = ('MEMFULL', 'MEMREAD', 'NOTSTABLE', 'INVALID', 'INVALIDLINE')
        Errors = namedtuple('Errors', errors, defaults=tuple(range(len(errors))))
        return Errors()
    ERRORS = errors.__func__()
    VARIABLES = {'RPM':1200,'SPINUP_TIME':1.5, 'STABLE_TIME':1.125, 'FACETS':4,
                 'CRYSTAL_HZ':50E6, 'LASER_HZ':100E3,
                 'END%':0.7, 'START%':0.35, 'SINGLE_LINE':False,
                 'SINGLE_FACET':False, 'DIRECTION':0}
    CHUNKSIZE = 1 # ONLY 1 is supported!!
    # one block is 4K bits, there are 32 blocks (officially 20 in HX4K)
    MEMWIDTH = 8
    MEMDEPTH = 512

    def __init__(self, platform, test=False):
        # variables
        self.ticksinfacet = round(self.VARIABLES['CRYSTAL_HZ']/(self.VARIABLES['RPM']/60*self.VARIABLES['FACETS']))
        LASERTICKS = int(self.VARIABLES['CRYSTAL_HZ']/self.VARIABLES['LASER_HZ'])
        self.JITTERTICKS = round(0.5*LASERTICKS)
        if self.VARIABLES['END%']>round(1-(self.JITTERTICKS+1)/self.ticksinfacet): raise Exception("Invalid settings, END% too high")
        self.BITSINSCANLINE = round((self.ticksinfacet*(self.VARIABLES['END%']-self.VARIABLES['START%']))/LASERTICKS)
        if self.BITSINSCANLINE <= 0: raise Exception("Bits in scanline invalid")
        self.bytesinline = math.ceil(self.BITSINSCANLINE/8)+1 # command byte is equal to 1
        if self.VARIABLES['SINGLE_LINE']==1:
            self.MEMDEPTH = self.bytesinline 
        elif (self.MEMWIDTH*self.MEMDEPTH)//self.BITSINSCANLINE<5:
            raise Exception("Memory too small for 5 lines")
        # clock; routing was not able to reach speed higher than 70 MHz
        #        for entire circuit on iCE40, so clock is contraint to 50 MHz
        clk100 = platform.request('clk100')
        self.clock_domains.cd_sys = ClockDomain(reset_less=True)
        platform.add_period_constraint(self.cd_sys.clk, 20)
        if not test:
            self.submodules.pll = pll = iCE40PLL()
            #self.comb += pll.reset.eq(~rst_n)
            pll.register_clkin(clk100, 100e6)
            pll.create_clkout(self.cd_sys, self.VARIABLES['CRYSTAL_HZ'])
        # three submodules; SPI receiver, memory and laser state machine
        # full byte state
        self.laserfsmstate = Signal(3)    # state laser module 5-8 bit
        self.error = Signal(5)            # error              0-5 bit
        debug = Signal(8)                 # optional 
        # Memory element
        # Rules:
        #        read cannot be set equal to write address  --> handled by laser ledfsm
        #        write cannot be set equal to read address  --> handled by receiver statemachne
        # readbit, current bit read
        # written to detect if already information is written to memory
        #         written can go zero after a write... if the memory is complety read --> it is no longer "written"
        # dat_r_temp , data is shifted after read. It is believed that this is not possible on the memory element
        # As a result data is copied to another element first.
        # sram memory is 32 blocks... each block has its own ports
        # one block 8*512 = 4096 bits currently used
        self.specials.mem = Memory(width=self.MEMWIDTH, depth=self.MEMDEPTH)
        writeport = self.mem.get_port(write_capable=True, mode = READ_FIRST)
        self.readport = self.mem.get_port(has_re=True)
        self.specials += writeport, self.readport
        self.ios = {writeport.adr, writeport.dat_w, writeport.we, self.readport.dat_r, self.readport.adr, self.readport.re}
        readbit = Signal(max = self.MEMWIDTH)
        written = Signal()
        self.dat_r_new = Signal(self.MEMWIDTH)
        dat_r_old = Signal(self.MEMWIDTH)
        # Receiver State Machine
        # TODO: tried to paralize receiver and parser but this gives errors
        #       I am not able achieve agreement between Linux host and FPGA on memfull
        #       Memfull is not correctly propagated every 10K operations which results in failure.
        # Consists out of component from litex and own custom component
        # Detects whether new command is available
        self.spi = platform.request("spi")
        spislave = SPISlave(self.spi, data_width=8)
        self.submodules.slave = spislave
        # Done detector
        done_d = Signal()
        done_rise = Signal()
        self.sync += done_d.eq(spislave.done)
        self.comb += done_rise.eq(spislave.done & ~done_d)
        # Start detector 
        start_d = Signal()
        start_rise = Signal()
        self.sync += start_d.eq(spislave.start)
        self.comb += start_rise.eq(spislave.start & ~start_d)
        # Memfull trigger
        dummyf = Signal(max=2*self.MEMDEPTH)
        dummyb = Signal(min=-self.MEMDEPTH, max=self.MEMDEPTH)
        self.sync += dummyf.eq(writeport.adr+self.CHUNKSIZE+1)
        self.sync += dummyb.eq(writeport.adr+self.CHUNKSIZE+1-(self.MEMDEPTH-1))
        self.submodules.parser = FSM(reset_state = "RESET")
        spi_mosi = Signal(self.MEMWIDTH)
        parsertrigger = Signal()
        memfulld = Signal()
        command = Signal()
        chunkcnt = Signal(max=self.CHUNKSIZE+1)
        self.parser.act("RESET",
            NextValue(command, 1),
            NextValue(chunkcnt, 0),
            NextState('WAITFORSTART')
        )
        self.parser.act("WAITFORSTART",
                If(start_rise,
                    NextState("WAITFORDONE")
                ).
                Else(
                    If((written==1)&(self.VARIABLES['SINGLE_LINE']==False),
                            If((dummyf>self.readport.adr)&(self.readport.adr>writeport.adr),
                                NextValue(self.error[self.ERRORS.MEMFULL],1)
                            ).
                            Elif((dummyb>self.readport.adr)&(self.readport.adr<writeport.adr),
                                NextValue(self.error[self.ERRORS.MEMFULL],1)
                            ).
                            Else(NextValue(self.error[self.ERRORS.MEMFULL],0))
                    ).
                    Else(NextValue(self.error[self.ERRORS.MEMFULL],0)),
                         NextValue(spislave.miso, Cat([self.error, self.laserfsmstate])),
                         NextValue(memfulld, self.error[self.ERRORS.MEMFULL])
                    )
        )       
        self.parser.act("WAITFORDONE",
            NextValue(spi_mosi, spislave.mosi),
            If(done_rise,
                #TODO: don't overfloat bug with not stable
                NextValue(spislave.miso, 4),  
                If(command,
                    NextState("PROCESSCOMMAND")
                ).
                Else(
                    If(chunkcnt>=self.CHUNKSIZE-1,
                        NextValue(chunkcnt, 0),
                        NextValue(command, 1)
                    ).
                    Else(NextValue(chunkcnt, chunkcnt+1)
                    ),
                    NextValue(command, 1),
                    NextValue(writeport.dat_w, spislave.mosi),
                    NextValue(writeport.we, 1),
                    NextState("WRITE")
                )
            )
        )
        # command is now used as extra parameter for state and makes it more confusing
        self.parser.act("PROCESSCOMMAND",
            NextState("WAITFORSTART"),
            If(spi_mosi == self.COMMANDS.STOP,
                NextValue(self.laserfsmstate, self.STATES.STOP)
            ).
            Elif(spi_mosi == self.COMMANDS.START,
                NextValue(self.laserfsmstate, self.STATES.START)
            ).
            Elif(spi_mosi == self.COMMANDS.LASERTEST,
                NextValue(self.laserfsmstate, self.STATES.LASERTEST)
            ).
            Elif(spi_mosi == self.COMMANDS.MOTORTEST,
                NextValue(self.laserfsmstate, self.STATES.MOTORTEST)
            ).
            Elif(spi_mosi == self.COMMANDS.LINETEST,
                NextValue(self.laserfsmstate, self.STATES.LINETEST)
            ).
            Elif(spi_mosi == self.COMMANDS.PHOTODIODETEST,
                NextValue(self.laserfsmstate, self.STATES.PHOTODIODETEST)
            ).
            Elif(spi_mosi == self.COMMANDS.WRITE_L,
                If(memfulld == 0 ,
                    NextValue(command, 0)
                )
            ).
            Elif(spi_mosi == self.COMMANDS.STATUS
            ).
            Elif(spi_mosi != 0,
                NextValue(self.error[self.ERRORS.INVALID], 1)
            )
        )
        self.parser.act("WRITE",
            NextState("WAITFORSTART"),
            NextValue(written, 1),
            # Write address position
            If(writeport.adr+1==self.MEMDEPTH,
               NextValue(writeport.adr, 0)
            ).
            # wrap around is different in single line mode
            Elif((writeport.adr==math.ceil(self.BITSINSCANLINE/self.MEMWIDTH))&(self.VARIABLES['SINGLE_LINE']==True), 
                NextValue(writeport.adr, 0)
            ).
            Else(
                NextValue(writeport.adr, writeport.adr+1)
            ),
            NextValue(writeport.we, 0)
        )
        # the original motor driver was designed for 6 facets and pulsed for eached facet
        polyperiod = int(self.VARIABLES['CRYSTAL_HZ']/(self.VARIABLES['RPM']/60)/(6*2))
        pwmcounter = Signal(max=polyperiod)
        self.poly_pwm = platform.request("poly_pwm")
        self.sync += If(pwmcounter == 0,
                self.poly_pwm.eq(~self.poly_pwm),
                pwmcounter.eq(polyperiod-1)).Else(
                pwmcounter.eq(pwmcounter - 1)
                )
        # Laser FSM
        # Laser FSM controls the laser, polygon anld output to motor
        self.facetcnt = Signal(max=self.VARIABLES['FACETS'])
        # stable counter used for both spinup and photo diode stable
        spinupticks = round(self.VARIABLES['SPINUP_TIME']*self.VARIABLES['CRYSTAL_HZ'])
        stableticks = round(self.VARIABLES['STABLE_TIME']*self.VARIABLES['CRYSTAL_HZ'])
        stablecounter = Signal(max=max(spinupticks, stableticks))   # counter is used twice, hence the max
        stablethresh = Signal(max=stableticks)
        self.lasercnt = Signal(max=LASERTICKS)
        self.scanbit = Signal(max=self.BITSINSCANLINE+1)
        self.tickcounter = Signal(max=int(self.ticksinfacet*2))
        self.submodules.laserfsm = FSM(reset_state = "RESET")
        # leesfoutdetectie:
        #   in het begin zijn lees en schrijfadres gelijk
        #   als er geschreven is dan is het schrijf adres een groter dan lees adres
        #   als er geschreven is en het volgende adres waarvan je gaat lezen nog niet beschreven is --> lees fout
        # op het moment kost lezen een tick, dit zou voorkomen kunnen worden
        # tick counter; number of ticks in a facet for the oscillator
        # laser counter; laser operates at reduced speed this controlled by this counter
        # readbit counter; current bit position in memory
        # scanbit counter; current bit position along scanline
        readtrig = Signal()
        self.submodules.readmem= FSM(reset_state = "RESET")
        self.readmem.act("RESET",
            NextValue(self.readport.adr, 0),
            NextValue(self.readport.re, 1),
            NextValue(written, 0),
            NextState("WAIT")
        )
        self.readmem.act("WAIT",
            If(readtrig,
               NextValue(self.readport.re, 0),
               NextState("READ")
            )
        )
        self.readmem.act("READ",
            NextValue(readtrig, 0),
            NextValue(self.readport.re, 1),
            If((self.readport.adr == writeport.adr)&(written == 0),
               NextValue(self.error[self.ERRORS.MEMREAD], 1),
            ).
            Else(
                #TODO: you could split into two signals --> one if ever memread occured, other memread signal
                NextValue(self.error[self.ERRORS.MEMREAD], 0),
                NextValue(self.dat_r_new, self.readport.dat_r)
            ),
            # always increase address, if you move over the write set written to zero
            If(self.readport.adr+1==self.MEMDEPTH,
                NextValue(self.readport.adr, 0),
                If((writeport.adr == 0)&(self.VARIABLES['SINGLE_LINE']==False), NextValue(written,0))
            ).
            Else(
                NextValue(self.readport.adr, self.readport.adr+1),
                If((self.readport.adr+1 == writeport.adr)&(self.VARIABLES['SINGLE_LINE']==False), NextValue(written,0))
            ),
            NextState("WAIT")
        )
        self.laserfsm.act("RESET",
            NextValue(self.readport.adr, 0),
            NextValue(writeport.adr, 0),
            NextState("STOP")
        )
        self.laser0 = platform.request("laser0")
        self.poly_en = platform.request("poly_en")
        self.photodiode = platform.request("photodiode")
        self.laserfsm.act("STOP",
            NextValue(stablethresh, stableticks-1),
            NextValue(stablecounter, 0),
            NextValue(self.facetcnt, 0),
            NextValue(self.tickcounter, 0),
            NextValue(self.scanbit, 0),
            NextValue(self.lasercnt, 0),
            NextValue(self.laser0, 0),
            NextValue(self.poly_en, 1),
            NextValue(readbit,0),
            If(self.laserfsmstate==self.STATES.START,
                If(self.photodiode == 0,
                    NextValue(self.laserfsmstate, self.STATES.STOP),
                    NextState("STOP")
                ).
                Else(
                    NextValue(readtrig, 1),
                    NextValue(self.error[self.ERRORS.NOTSTABLE], 0),
                    NextValue(self.error[self.ERRORS.MEMREAD], 0),
                    NextValue(self.poly_en, 0),
                    NextState("SPINUP")
                )
            ).
            Elif(self.laserfsmstate==self.STATES.MOTORTEST,
                NextValue(self.poly_en, 0),
                NextState("MOTORTEST")
            ).
            Elif(self.laserfsmstate==self.STATES.LASERTEST,
                NextValue(self.laser0, 1),
                NextState("LASERTEST")
            ).
            Elif(self.laserfsmstate==self.STATES.LINETEST,
                NextValue(self.laser0, 1),
                NextValue(self.poly_en, 0),
                NextState("LINETEST")
            ).
            Elif(self.laserfsmstate==self.STATES.PHOTODIODETEST,
                # photodiode should be high with laser off
                # something is wrong, this makes sure error is produced
                If(self.photodiode == 0,
                    NextValue(self.laser0, 1),
                    NextValue(self.poly_en, 1)
                ).
                Else(
                    NextValue(self.laser0, 1),
                    NextValue(self.poly_en, 0),
                ),
                NextState("PHOTODIODETEST")
            )
        )
        self.laserfsm.act("MOTORTEST",
            If(self.laserfsmstate!=self.STATES.MOTORTEST,
                 NextState("STOP")
            )
        )
        self.laserfsm.act("LASERTEST",
            If(self.laserfsmstate!=self.STATES.LASERTEST,
                 NextState("STOP")
            )
        )
        self.laserfsm.act("LINETEST",
            If(self.laserfsmstate!=self.STATES.LINETEST,
                 NextState("STOP")
            )
        )
        # Photodiode rising edge detector
        photodiode_d = Signal()
        self.laserfsm.act("PHOTODIODETEST",
            If((self.photodiode == 0) & (self.poly_en == 0),
                NextValue(self.laserfsmstate, self.STATES.STOP),
                NextState("STOP")
            ).
            Elif(self.laserfsmstate!=self.STATES.PHOTODIODETEST,
                 NextState("STOP")
            )
        )
        self.laserfsm.act("SPINUP",
            NextValue(stablecounter, stablecounter + 1),
            If(stablecounter>spinupticks-1,
                NextState("STATE_WAIT_STABLE"),
                NextValue(self.laser0, 1),
                NextValue(stablecounter, 0),
            )
        )
        self.laserfsm.act("STATE_WAIT_STABLE",
            NextValue(stablecounter, stablecounter+1),
            NextValue(photodiode_d, self.photodiode),
            If(stablecounter>=stablethresh,
               NextValue(self.error[self.ERRORS.NOTSTABLE], 1),
               NextValue(self.laserfsmstate, self.STATES.STOP),
               NextState('RESET')
            ).
            Elif(~self.photodiode&~photodiode_d,
               NextValue(self.tickcounter, 0),
               NextValue(self.laser0, 0),
               If((self.tickcounter>self.ticksinfacet-self.JITTERTICKS)&
                  (self.tickcounter<self.ticksinfacet+self.JITTERTICKS),
                  If(self.facetcnt==self.VARIABLES['FACETS']-1, NextValue(self.facetcnt, 0)).
                  Else(NextValue(self.facetcnt, self.facetcnt+1)),
                  NextValue(stablecounter, 0),
                  If((self.VARIABLES['SINGLE_FACET']==True)&(self.facetcnt>0),
                    NextState('WAIT_END')
                  ).
                  Else(
                    NextValue(stablethresh, min(round(10.1*self.ticksinfacet), stableticks)),  #TODO: lower!
                    NextState('READ_INSTRUCTION')
                  )
               ).
               Else(
                   NextState('WAIT_END')
               )
            ).
            Elif(self.laserfsmstate!=self.STATES.START,
                 NextState("RESET")
            ).  
            Else(
                NextValue(self.tickcounter, self.tickcounter+1)
            )
        )
        self.laserfsm.act('READ_INSTRUCTION',
            NextValue(self.tickcounter, self.tickcounter+1),
            If(readtrig == 0,
               If(self.error[self.ERRORS.MEMREAD] == 1,
                   # move back the address and read again
                   If(self.readport.adr == 0,
                        NextValue(self.readport.adr, self.MEMDEPTH-1)
                   ).
                   Else(
                       NextValue(self.readport.adr, self.readport.adr-1)
                   ),
                   NextValue(readtrig, 1),
                   NextState("WAIT_END")
               ).
               Elif(self.dat_r_new == self.INSTRUCTIONS.STOP,
                   NextState("RESET"),
                   NextValue(self.laserfsmstate, self.STATES.STOP)
               ).
               Elif(self.dat_r_new == self.INSTRUCTIONS.SCAN,
                   NextState('WAIT_FOR_DATA_RUN'),
                   NextValue(readtrig, 1),
               ).
               Else(NextState("RESET"),
                    NextValue(self.laserfsmstate, self.STATES.STOP),
                    NextValue(self.error[self.ERRORS.INVALIDLINE], 1),
               )
            )
        )
        self.laserfsm.act('WAIT_FOR_DATA_RUN',
            NextValue(self.tickcounter, self.tickcounter + 1),
            If(readtrig == 0,
                If(self.error[self.ERRORS.MEMREAD] == 1,
                    NextValue(dat_r_old, 0)
                    ).
                Else(
                    NextValue(dat_r_old, self.dat_r_new)
                ),
                NextValue(readbit, 0),
                NextValue(self.scanbit, 0),
                NextValue(self.lasercnt, 0),
                If(self.tickcounter==int(self.VARIABLES['START%']*self.ticksinfacet-2),
                    NextState('DATA_RUN')
                ).
                Elif(self.tickcounter>int(self.VARIABLES['START%']*self.ticksinfacet-2),
                    #NextValue(self.error[self.ERRORS.INVALID], 1),   #TODO: replace with timeout
                    NextState('DATA_RUN')
                )
            )
        )
        self.laserfsm.act("DATA_RUN",
            NextValue(self.tickcounter, self.tickcounter + 1),
            If(self.lasercnt == 0,
                #NOTE: readbit and scanbit counters can be different
                #      readbit is your current position in memory and scanbit your current byte position in scanline
                If(self.scanbit >= self.BITSINSCANLINE,
                    NextState("WAIT_END")
                ).
                Else(
                    NextValue(self.lasercnt, LASERTICKS-1),
                    NextValue(self.scanbit, self.scanbit+1),
                    # read from memory before the spinup
                    # it is triggered here again, so fresh data is available once the end is reached
                    # if read bit is 0, trigger a read out unless the next byte is outside of line
                    If(readbit == 0,
                        NextValue(readtrig, 1),
                        NextValue(readbit, readbit+1),
                        NextValue(dat_r_old, dat_r_old>>1)
                    ).
                    # final read bit copy memory
                    # move to next address, i.e. byte, if end is reached
                    Elif(readbit==self.MEMWIDTH-1,
                        If(self.error[self.ERRORS.MEMREAD] == 1,
                            NextValue(dat_r_old, 0)
                        ).
                        Else(
                            NextValue(dat_r_old, self.dat_r_new)
                        ),
                        NextValue(readbit, 0)
                    ).
                    Else(
                        NextValue(readbit, readbit+1),
                        NextValue(dat_r_old, dat_r_old>>1)
                    ),
                    NextValue(self.laser0, dat_r_old[0])
                )
            ).
            Else(
                NextValue(self.lasercnt, self.lasercnt - 1)
            )
        )
        self.laserfsm.act("WAIT_END",
            NextValue(stablecounter, stablecounter+1),
            NextValue(self.tickcounter, self.tickcounter+1),
            If(self.tickcounter>=round(self.ticksinfacet-self.JITTERTICKS-1),
               NextState("STATE_WAIT_STABLE"),
               NextValue(self.laser0, 1),
            )
        )