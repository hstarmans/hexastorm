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
        commands = ('RECVCOMMAND', 'STATUS', 'START', 'STOP', 'LASERTEST',
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
        errors = ('MEMFULL', 'MEMREAD', 'NOTSTABLE', 'INVALID', 'INVALIDLINE', 'TIMEOUT')
        Errors = namedtuple('Errors', errors, defaults=tuple(range(len(errors))))
        return Errors()
    ERRORS = errors.__func__()
    VARIABLES = {'RPM':1200,'SPINUP_TIME':1.5, 'STABLE_TIME':1.125, 'FACETS':4,
                 'CRYSTAL_HZ':50E6, 'LASER_HZ': 100E3,
                 'END%': 0.7, 'START%': 0.35, 'SINGLE_LINE':False,
                 'SINGLE_FACET':False, 'DIRECTION':0}
    CHUNKSIZE = 1 # ONLY 1 is supported!!
    # one block is 4K bits, there are 32 blocks (officially 20 in HX4K)
    MEMWIDTH = 8  
    MEMDEPTH = 512
    assert (MEMDEPTH%8 == 0) & (MEMDEPTH>=8)


    def __init__(self, platform, test=False):
        # variables
        self.ticksinfacet = round(self.VARIABLES['CRYSTAL_HZ']/(self.VARIABLES['RPM']/60*self.VARIABLES['FACETS']))
        LASERTICKS = int(self.VARIABLES['CRYSTAL_HZ']/self.VARIABLES['LASER_HZ'])
        self.JITTERTICKS = round(0.5*LASERTICKS)
        if self.VARIABLES['END%']>(1-self.JITTERTICKS/self.ticksinfacet): raise Exception("Invalid settings, END% too high")
        self.BITSINSCANLINE = round((self.ticksinfacet*(self.VARIABLES['END%']-self.VARIABLES['START%']))/LASERTICKS)
        if self.BITSINSCANLINE <= 0: raise Exception("Bits in scanline invalid")
        if (self.MEMWIDTH*self.MEMDEPTH)//self.BITSINSCANLINE<5: raise Exception("Memory too small for 5 lines")
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
        self.writebyte = Signal(max=self.MEMDEPTH)
        written = Signal()
        self.dat_r_new = Signal(max= self.MEMWIDTH)
        dat_r_old = Signal(max= self.MEMWIDTH)
        # Receiver State Machine
        # Consists out of component from litex and own custom component
        # Detects whether new command is available
        self.spi = platform.request("spi")
        spislave = SPISlave(self.spi, data_width=8)
        self.submodules.slave = spislave
        # COMMANDS 
        # The command variable contains command to be executed
        # typically the recvcommand, cannot be set externally
        command = Signal(max=len(self.COMMANDS))
        # Done detector
        done_d = Signal()
        done_rise = Signal()
        self.sync += done_d.eq(spislave.done)
        self.comb += done_rise.eq(spislave.done & ~done_d)
        # Start detector (could be refactored)
        start_d = Signal()
        start_rise = Signal()
        self.sync += start_d.eq(spislave.start)
        self.comb += start_rise.eq(spislave.start & ~start_d)
        self.sync += self.error[self.ERRORS.MEMFULL].eq((writeport.adr==self.readport.adr)&(written==1)&(self.VARIABLES['SINGLE_LINE']==False))
        # Custom Receiver
        self.submodules.receiver = FSM(reset_state = "IDLE")
        self.receiver.act("IDLE",
                NextValue(spislave.miso, Cat([self.error, self.laserfsmstate])),
                If(start_rise,
                    NextState("WAITFORDONE")
                )
        )
        self.receiver.act("WAITFORDONE",
            If(done_rise,
                NextState("PROCESSINPUT")
            )
        )
        self.receiver.act("PROCESSINPUT",
            NextState("IDLE"),
            # Read Header
            If(command == self.COMMANDS.RECVCOMMAND,
                If(spislave.mosi == self.COMMANDS.STOP,
                    NextValue(self.laserfsmstate, self.STATES.STOP)
                ).
                Elif(spislave.mosi == self.COMMANDS.START,
                    NextValue(self.laserfsmstate, self.STATES.START)
                ).
                Elif(spislave.mosi == self.COMMANDS.LASERTEST,
                    NextValue(self.laserfsmstate, self.STATES.LASERTEST)
                ).
                Elif(spislave.mosi == self.COMMANDS.MOTORTEST,
                    NextValue(self.laserfsmstate, self.STATES.MOTORTEST)
                ).
                Elif(spislave.mosi == self.COMMANDS.LINETEST,
                    NextValue(self.laserfsmstate, self.STATES.LINETEST)
                ).
                Elif(spislave.mosi == self.COMMANDS.PHOTODIODETEST,
                    NextValue(self.laserfsmstate, self.STATES.PHOTODIODETEST)
                ).
                Elif(spislave.mosi == self.COMMANDS.READ_D,
                    NextValue(command, self.COMMANDS.READ_D),
                    #NOTE doesn't work as you jump to idle where miso is changed
                    NextValue(spislave.miso, debug)
                ).
                Elif(spislave.mosi == self.COMMANDS.WRITE_L,
                    # only switch to write stage if memory is not full
                    If((writeport.adr==self.readport.adr)&(written==1),
                        NextValue(command, self.COMMANDS.RECVCOMMAND)
                    ).
                    Else(
                        NextValue(command, self.COMMANDS.WRITE_L)
                    )
                ).
                Elif(spislave.mosi == self.COMMANDS.STATUS).
                Elif(spislave.mosi != 0,
                    NextValue(self.error[self.ERRORS.INVALID], 1)
                )
                # TODO: there seem to be zero commands being sent over?
                # uncaptured
            ).
            # Read data after header; only applicable for debug or write line
            Else(
                If(command == self.COMMANDS.READ_D,
                    NextValue(command, self.COMMANDS.RECVCOMMAND),
                ).
                # command must be WRITE_L
                Else(
                    NextValue(written, 1),
                    NextValue(writeport.dat_w, spislave.mosi),
                    NextValue(writeport.we, 1),
                    NextState("WRITE"),
                    If(self.writebyte>=self.CHUNKSIZE-1,
                        NextValue(self.writebyte, 0),
                        NextValue(command, self.COMMANDS.RECVCOMMAND)
                    ).
                    Else(NextValue(self.writebyte, self.writebyte+1)
                    )
                )
            )
        )
        self.receiver.act("WRITE",
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
            NextValue(writeport.we, 0),
            NextState("IDLE")
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
        # scanbit counter; current bit positioin along scanline
        readtrig = Signal()
        self.submodules.readmem= FSM(reset_state = "RESET")
        self.readmem.act("RESET",
            NextValue(writeport.adr, 0),
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
                NextValue(self.dat_r_new, self.readport.dat_r),
                # increase address after succesfull read
                If(self.readport.adr+1==self.MEMDEPTH,
                   NextValue(self.readport.adr, 0),
                   If(writeport.adr == 0, NextValue(written,0))
                ).
                Else(
                    NextValue(self.readport.adr, self.readport.adr+1),
                    If(self.readport.adr+1 == writeport.adr, NextValue(written,0))
                )
            ),
            NextState("WAIT")
        )
        self.laserfsm.act("RESET",
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
            ),
            If(self.laserfsmstate!=self.STATES.START,
                 NextState("STOP")
            )
        )
        self.laserfsm.act("STATE_WAIT_STABLE",
            NextValue(stablecounter, stablecounter+1),
            NextValue(photodiode_d, self.photodiode),
            If(stablecounter>=stablethresh,
               NextValue(self.error[self.ERRORS.NOTSTABLE], 1),
               NextValue(self.laserfsmstate, self.STATES.STOP),
               NextState('STOP')
            ).
            Elif(~self.photodiode&~photodiode_d,
               NextValue(self.tickcounter, 0),
               NextValue(self.laser0, 0),
               If((self.tickcounter>self.ticksinfacet-self.JITTERTICKS)&
                  (self.tickcounter<self.ticksinfacet+self.JITTERTICKS),
                  If(self.facetcnt==0, NextValue(self.facetcnt, self.VARIABLES['FACETS']-1)).
                  Else(NextValue(self.facetcnt, self.facetcnt-1)), 
                  NextValue(stablecounter, 0),
                  NextValue(stablethresh, round(1.1*self.ticksinfacet)), 
                  If((self.VARIABLES['SINGLE_FACET']==True)&(self.facetcnt>0),
                    NextState('WAIT_END')   # WAIT END
                  ).
                  Else(
                    NextState('READ_INSTRUCTION'),
                    NextValue(readtrig, 1),
                  )
               ).
               Else(
                   NextState('WAIT_END')  # WAIT END
               )
            ).
            Elif(self.laserfsmstate!=self.STATES.START,
                 NextState("STOP")
            ).  
            Else(
                NextValue(self.tickcounter, self.tickcounter+1)
            )
        )
        self.laserfsm.act('READ_INSTRUCTION',
            NextValue(self.tickcounter, self.tickcounter+1),
            If(readtrig == 0,
               If(self.error[self.ERRORS.MEMREAD] == 1,
                   NextState("WAIT_END"),
               ).
               Elif(self.dat_r_new == self.INSTRUCTIONS.STOP,
                   NextState("STOP"),
                   NextValue(self.laserfsmstate, self.STATES.STOP),
               ).
               Elif(self.dat_r_new == self.INSTRUCTIONS.SCAN,
                   NextState('WAIT_FOR_DATA_RUN'),
                   NextValue(readtrig, 1),
               ).
               Else(NextState("WAIT_END"),
                   NextValue(self.error[self.ERRORS.INVALIDLINE], 1),
               )
            )
        )
        self.laserfsm.act('WAIT_FOR_DATA_RUN',
            NextValue(self.tickcounter, self.tickcounter + 1),
            If(readtrig == 0,
                NextValue(dat_r_old, self.dat_r_new),
                NextValue(readbit, 0),
                If(self.tickcounter==int(self.VARIABLES['START%']*self.ticksinfacet-2),
                    NextState('DATA_RUN')
                ).
                Elif(self.tickcounter>int(self.VARIABLES['START%']*self.ticksinfacet-2),
                    #NextValue(self.error[self.ERRORS.INVALID], 1),   #TODO: replace with timeout
                    NextState('DATA_RUN')
                )
            ).
            Elif(self.laserfsmstate != self.STATES.START,
                 NextState("STOP")
            )
        )
        self.laserfsm.act("DATA_RUN",
            NextValue(self.tickcounter, self.tickcounter + 1),
            If(self.lasercnt == 0,
                #NOTE: readbit and scanbit counters can be different
                #      readbit is your current position in memory and scanbit your current byte position in scanline
                If(self.scanbit >= self.BITSINSCANLINE,
                    NextState("WAIT_END"),
                    NextValue(self.scanbit, 0),
                    If(self.VARIABLES['SINGLE_LINE']==1,
                        NextValue(self.error[self.ERRORS.MEMREAD], 0),
                        NextValue(self.readport.adr, 0)  # this is tricky as you don't execute written check
                    )
                ).
                Else(
                    NextValue(self.lasercnt, LASERTICKS-1),
                    NextValue(self.scanbit, self.scanbit+1),
                    # read from memory before the spinup
                    # it is triggered here again, so fresh data is available once the end is reached
                    # if read bit is 0, trigger a read out unless the next byte is outside of line
                    If((readbit == 0)&((self.scanbit+9)<self.BITSINSCANLINE),
                        NextValue(readtrig, 1),
                        NextValue(readbit, readbit+1),
                        NextValue(dat_r_old, dat_r_old>>1)
                    ).
                    # final read bit copy memory
                    # move to next address, i.e. byte, if end is reached
                    Elif(readbit==self.MEMWIDTH-1,
                        NextValue(dat_r_old, self.dat_r_new),
                        NextValue(readbit, 0)
                    ).
                    Else(
                        NextValue(readbit, readbit+1),
                        NextValue(dat_r_old, dat_r_old>>1)
                    ),
                    # if there is no data, laser off error should already have been reported
                    If(self.error[self.ERRORS.MEMREAD] == 1,
                        NextValue(self.laser0, 0)
                    ).
                    # else use the data from the memory
                    Else(NextValue(self.laser0, dat_r_old[0]))
                )
            ).
            Elif(self.laserfsmstate!=self.STATES.START,
               NextState("STOP")
            ).
            Else(
                NextValue(self.lasercnt, self.lasercnt - 1)
            )
        )
        self.laserfsm.act("WAIT_END",
            NextValue(self.tickcounter, self.tickcounter+1),
            If(self.tickcounter>=self.ticksinfacet-2*self.JITTERTICKS,
               NextState("STATE_WAIT_STABLE"),
               NextValue(self.laser0, 1),
            )
        )