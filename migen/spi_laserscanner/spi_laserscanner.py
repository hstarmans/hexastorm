"""
    spi_laserscanner.py
    Core module for the laser scanner

    Rik Starmans
"""
import sys
from collections import namedtuple

import unittest
from migen.fhdl.tools import list_special_ios
from migen import *
from litex.soc.cores.spi import SPISlave

#

sys.path.append("..") 
import hexa as board

# lines that can be in memory
# LINES = (LEDPROGRAM.MEMWIDTH*LEDPROGRAM.MEMDEPTH)//VARIABLES['SCANLINE_DATA_SIZE']


class Scanhead(Module):
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
    def errors():
        '''errors
        Multiple errors can be present at the same time. This is different than state or command
        this always one.
        If all bits are zero, there is no error.
        Returned is the bit which equals the error
        '''
        errors = ('MEMFULL', 'MEMREAD', 'NOTSTABLE', 'INVALID')
        Errors = namedtuple('Errors', errors, defaults=tuple(range(len(errors))))
        return Errors()
    ERRORS = errors.__func__()

    #TODO: SYNC START and SINGLE_FACET are not used
    #      SYNC_START timestamp at which you turn on the laser
    VARIABLES = {'RPM':2400,'SPINUP_TIME':1.5, 'STABLE_TIME':1.125, 'FACETS':4,
            'CRYSTAL_HZ':100E6, 'LASER_HZ': 100E3,
            'END%': 0.8, 'START%': 0.35, 'SINGLE_LINE':False,
            'SINGLE_FACET':False, 'DIRECTION':0, 'SYNCSTART':1/3000, 'JITTER_THRESH':1/400}
    CHUNKSIZE = 8 # you write in chunks of 8 bytes
    # one block is 4K bits, there are 32 blocks (officially 20 in HX4K)
    MEMWIDTH = 8  
    MEMDEPTH = 512


    def __init__(self, spi_port, laser0, poly_pwm, poly_en,
                 photodiode):
        # three submodules; SPI receiver, memory and laser state machine
        # full byte state
        self.laserfsmstate = Signal(3)    # state laser module 6-8 byte
        self.error = Signal(5)            # error              0-5 byte
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
        readport = self.mem.get_port(has_re=True)
        self.specials += writeport, readport
        self.ios = {writeport.adr, writeport.dat_w, writeport.we, readport.dat_r, readport.adr, readport.re}
        readbit = Signal(max = self.MEMWIDTH)
        self.writebyte = Signal(max=self.MEMDEPTH)
        written = Signal()
        dat_r_new = Signal(max= self.MEMWIDTH)
        dat_r_old = Signal(max= self.MEMWIDTH)
        # Receiver State Machine
        # Consists out of component from litex and own custom component
        # Detects whether new command is available
        spislave = SPISlave(spi_port, data_width=8)
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
        self.sync += self.error[self.ERRORS.MEMFULL].eq((writeport.adr==readport.adr)&(written==1))
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
                    NextValue(self.error[self.ERRORS.INVALID], 0),
                    NextValue(self.laserfsmstate, self.STATES.STOP)
                ).
                Elif(spislave.mosi == self.COMMANDS.START,
                    NextValue(self.error[self.ERRORS.INVALID], 0),
                    NextValue(self.laserfsmstate, self.STATES.START)
                ).
                Elif(spislave.mosi == self.COMMANDS.LASERTEST,
                    NextValue(self.error[self.ERRORS.INVALID], 0),
                    NextValue(self.laserfsmstate, self.STATES.LASERTEST)
                ).
                Elif(spislave.mosi == self.COMMANDS.MOTORTEST,
                    NextValue(self.error[self.ERRORS.INVALID], 0),
                    NextValue(self.laserfsmstate, self.STATES.MOTORTEST)
                ).
                Elif(spislave.mosi == self.COMMANDS.LINETEST,
                    NextValue(self.error[self.ERRORS.INVALID], 0),
                    NextValue(self.laserfsmstate, self.STATES.LINETEST)
                ).
                Elif(spislave.mosi == self.COMMANDS.PHOTODIODETEST,
                    NextValue(self.error[self.ERRORS.INVALID], 0),
                    NextValue(self.laserfsmstate, self.STATES.PHOTODIODETEST)
                ).
                Elif(spislave.mosi == self.COMMANDS.READ_D,
                    NextValue(self.error[self.ERRORS.INVALID], 0),
                    NextValue(command, self.COMMANDS.READ_D),
                    #NOTE doesn't work as you jump to idle where miso is changed
                    NextValue(spislave.miso, debug)
                ).
                Elif(spislave.mosi == self.COMMANDS.WRITE_L,
                    # only switch to write stage if memory is not full
                    NextValue(self.error[self.ERRORS.INVALID], 0),
                    If((writeport.adr==readport.adr)&(written==1),
                        NextValue(command, self.COMMANDS.RECVCOMMAND)
                    ).
                    Else(
                        NextValue(command, self.COMMANDS.WRITE_L)
                    )
                ).
                Elif(spislave.mosi == self.COMMANDS.STATUS,
                     NextValue(self.error[self.ERRORS.INVALID], 0)
                ).
                Elif(spislave.mosi != 0,
                    NextValue(self.error[self.ERRORS.INVALID], 1)
                )
                # TODO: there seem to be zero commands beeing sent over?
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
            NextValue(writeport.adr, writeport.adr+1), 
            NextValue(writeport.we, 0),
            NextState("IDLE")
        )
        # 
        polyperiod = int(self.VARIABLES['CRYSTAL_HZ']/(self.VARIABLES['RPM']*60))
        pwmcounter = Signal(max=polyperiod)
        self.sync += If(pwmcounter == 0,
                poly_pwm.eq(~poly_pwm),
                pwmcounter.eq(int(polyperiod))).Else(
                pwmcounter.eq(pwmcounter - 1)
                )
        # Laser FSM
        # Laser FSM controls the laser, polygon and output to motor
        facetcnt = Signal(max=int(self.VARIABLES['FACETS']))
        # stable counter used for both spinup and photo diode stable
        spinupticks = int(self.VARIABLES['SPINUP_TIME']*self.VARIABLES['CRYSTAL_HZ'])
        stableticks = int(self.VARIABLES['STABLE_TIME']*self.VARIABLES['CRYSTAL_HZ'])
        stablecounter = Signal(max=max(spinupticks, stableticks))
        ticksinfacet = self.VARIABLES['CRYSTAL_HZ']/(self.VARIABLES['RPM']*60*self.VARIABLES['FACETS'])
        LASERTICKS = int(self.VARIABLES['CRYSTAL_HZ']/self.VARIABLES['LASER_HZ'])
        BITSINSCANLINE = round((ticksinfacet*(self.VARIABLES['END%']-self.VARIABLES['START%']))/LASERTICKS)
        if BITSINSCANLINE <= 0: raise Exception("Bits in scanline invalid")            
        self.lasercnt = Signal(max=LASERTICKS)
        self.scanbit = Signal(max=BITSINSCANLINE+1)
        self.tickcounter = Signal(max=int(ticksinfacet*10))
        self.submodules.laserfsm = FSM(reset_state = "RESET")
        firststart = Signal()
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
            NextValue(readport.re, 0),
            If(readtrig,
               NextState("READ")
            )
        )
        self.readmem.act("READ",
            If((readport.adr == writeport.adr)&(written == 0),
               NextValue(readtrig, 0),
               NextValue(self.error[self.ERRORS.MEMREAD], 1),
            ).
            Else(
                NextValue(readtrig,0),
                NextValue(readport.re, 1),
                NextValue(dat_r_new, readport.dat_r),
                # increase address after succesfull read
                If(self.VARIABLES['SINGLE_LINE']==False,
                   NextValue(readport.adr, readport.adr+1),
                   If(readport.adr+1==writeport.adr,
                      NextValue(written,0)
                   ).
                   #NOTE: count wrap around
                   Elif((readport.adr+1==self.MEMDEPTH)&(writeport.adr==0),
                        NextValue(written,0)
                   )
                )
            ),
            NextState("RESET")
        )
        self.laserfsm.act("RESET",
            NextValue(firststart, 1),
            NextState("STOP")
        ) 
        self.laserfsm.act("STOP",
            NextValue(stablecounter, 0),
            NextValue(self.scanbit, 0),
            NextValue(self.lasercnt, 0),
            NextValue(laser0, 0),
            NextValue(poly_en, 1),
            NextValue(readbit,0),
            If(self.laserfsmstate==self.STATES.START,
                 NextValue(self.error[self.ERRORS.NOTSTABLE], 0),
                 NextValue(self.error[self.ERRORS.MEMREAD], 0),
                 NextValue(poly_en, 0),
                 If((firststart==1)|(self.error[self.ERRORS.MEMREAD]==1),
                    NextValue(readtrig, 1),
                    NextState("WAITREAD")
                 ).
                 Else(
                    NextState("SPINUP")
                 )
            ).
            Elif(self.laserfsmstate==self.STATES.MOTORTEST,
                NextValue(poly_en, 0),
                NextState("MOTORTEST")
            ).
            Elif(self.laserfsmstate==self.STATES.LASERTEST,
                NextValue(laser0, 1),
                NextState("LASERTEST")
            ).
            Elif(self.laserfsmstate==self.STATES.LINETEST,
                NextValue(laser0, 1),
                NextValue(poly_en, 0),
                NextState("LINETEST")
            ).
            Elif(self.laserfsmstate==self.STATES.PHOTODIODETEST,
                NextValue(laser0, 1),
                NextValue(poly_en, 0),
                NextState("PHOTODIODETEST")
            )
        )
        # if you start laser head for the first time, you need to read
        # in the first start there is no data in memory yet so you need to copy it
        self.laserfsm.act("WAITREAD",
            If(readtrig == 0,
               NextValue(firststart, 0),
               NextValue(dat_r_old, dat_r_new),
               NextState("SPINUP")
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
        self.laserfsm.act("PHOTODIODETEST",
            If(photodiode == 1,
               NextValue(laser0, 0),
               NextValue(poly_en, 1),
            ),
            If(self.laserfsmstate!=self.STATES.PHOTODIODETEST,
                 NextState("STOP")
            )
        )
        self.laserfsm.act("SPINUP",
            NextValue(stablecounter, stablecounter + 1),
            If(stablecounter>spinupticks-1,
                NextState("STATE_WAIT_STABLE"),
                NextValue(stablecounter, 0),
            ),
            If(self.laserfsmstate!=self.STATES.START,
                 NextState("STOP")
            )
        )
        # Photodiode falling edge detector
        photodiode_d = Signal()
        photodiode_fall = Signal()
        self.sync += photodiode_d.eq(photodiode)
        self.comb += photodiode_fall.eq(~photodiode & photodiode_d)
        #NOTE: in the following states... TICKCOUNTER MUST ALWAYS BE INCREASED
        self.laserfsm.act("STATE_WAIT_STABLE",
            NextValue(laser0, 1),
            NextValue(stablecounter, stablecounter+1),
            If(photodiode_fall,
               NextValue(facetcnt, facetcnt+1), 
               NextValue(self.tickcounter, 0),
               If((self.tickcounter+1>round(ticksinfacet*(1-self.VARIABLES['JITTER_THRESH'])))&
                  (self.tickcounter+1<round(ticksinfacet*(1+self.VARIABLES['JITTER_THRESH']))),
                  If((self.VARIABLES['SINGLE_FACET']==True)&(facetcnt>0),
                    NextState('WAIT_END')
                  ).
                  Else(
                    NextState('WAIT_FOR_DATA_RUN')
                  )
               )
            ).
            Else(
                NextValue(self.tickcounter, self.tickcounter+1)
            ),
            If(stablecounter>stableticks-1,
               NextValue(self.error[self.ERRORS.NOTSTABLE], 1),
               NextValue(self.laserfsmstate, self.STATES.STOP),
               NextState('STOP')
            ),
            If(self.laserfsmstate!=self.STATES.START,
                 NextState("STOP")
            )  
        )
        self.laserfsm.act('WAIT_FOR_DATA_RUN',
            NextValue(laser0, 0),
            NextValue(self.tickcounter, self.tickcounter+1),
            If(self.tickcounter>=int(self.VARIABLES['START%']*ticksinfacet-1),
                NextState('DATA_RUN')
            ),
            If(self.laserfsmstate!=self.STATES.START,
                 NextState("STOP")
            )
        )
        self.laserfsm.act("DATA_RUN",
            NextValue(self.tickcounter, self.tickcounter+1),
            If(self.lasercnt >= LASERTICKS-1,
                NextValue(self.lasercnt, 0),
                #NOTE: readbit and scanbit counters can be different
                #      readbit is your current position in memory and scanbit your current byte position in scanline
                If(self.scanbit >= BITSINSCANLINE,
                    NextState("WAIT_END"), 
                    NextValue(self.scanbit, 0)
                ).
                Else(
                    # if read bit is 0, trigger a read out
                    If(readbit==0,
                        NextValue(readtrig, 1),
                    ).
                    # final read bit copy memory
                    # move to next address, i.e. byte, if end is reached
                    Elif(readbit==self.MEMWIDTH-1,
                        NextValue(dat_r_old, dat_r_new),
                    ),
                    NextValue(dat_r_old, dat_r_old>>1),
                    # if there is no data, laser off error should already have been reported
                    If(written==0,
                        NextValue(laser0, 0)
                    ).
                    Else(NextValue(laser0, dat_r_old[0]),
                         NextValue(readbit, readbit+1),
                         NextValue(self.scanbit, self.scanbit+1)
                    )
                )
            ).
            Else(
                NextValue(self.lasercnt, self.lasercnt+1)
            ),
            If(self.laserfsmstate!=self.STATES.START,
               NextState("STOP")
            )
        )
        self.laserfsm.act("WAIT_END",
            NextValue(self.tickcounter, self.tickcounter+1),
            If(self.tickcounter>=round(self.VARIABLES['END%']*ticksinfacet-1),
               NextState("STATE_WAIT_STABLE")
            )
        )




if __name__ == '__main__':
    if len(sys.argv)>1:
        if sys.argv[1] == 'build':
            plat = board.Platform()
            spi_port = plat.request("spi")
            laser0 = plat.request("laser0")
            poly_pwm = plat.request("poly_pwm")
            poly_en = plat.request("poly_en")
            photodiode = plat.request("photodiode")
            spi_statemachine = Scanhead(spi_port, laser0, poly_pwm, poly_en, photodiode)
            plat.build(spi_statemachine, build_name = 'spi_statemachine')
    else:
        import laserscanner_test
        suite = unittest.TestLoader().loadTestsFromModule(laserscanner_test)
        unittest.TextTestRunner(verbosity=2).run(suite)