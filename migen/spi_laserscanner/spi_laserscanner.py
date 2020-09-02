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

#TODO: you could lower the tick counter


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

    VARIABLES = {'RPM':2400,'SPINUP_TIME':1.5, 'STABLE_TIME':1.125, 'FACETS':4,
            'CRYSTAL_HZ':100E6, 'SCANLINE_DATA_SIZE':790, 'TICKS_START':4375,
            'SINGLE_FACET':0, 'DIRECTION':0, 'SYNCSTART':1/400, 'JITTER_THRESH':1/3000}
    CHUNKSIZE = 8 # you write in chunks of 8 bytes
    MEMWIDTH = 8  # must be 8, as you receive in terms of eight
    MEMDEPTH = 512

    def __init__(self, spi_port, laser0, poly_pwm, poly_en,
                 photodiode):
        # three submodules; SPI receiver, memory and laser state machine
        # full byte state
        self.laserfsmstate = Signal(3)    # state laser module 6-8 byte
        self.error = Signal(5)      # error state  2-5 byte,
                                    # time out     2 byte 
                                    # read error   1 byte
                                    # memory full  0 byte
        debug = Signal(8)   # optional 
        # Memory element
        # Rules:
        #        read cannot be set equal to write address  --> handled by laser ledfsm
        #        write cannot be set equal to read address  --> handled by receiver statemachne
        # readbit, current bit read
        # written to detect if already information is written to memory
        # dat_r_temp , data is shifted after read. It is believed that this is not possible on the memory element
        # As a result data is copied to another element first.
        # sram memory is 32 blocks... each block has its own ports
        # one block 8*512 = 4096 bits currently used
        self.specials.mem = Memory(width=self.MEMWIDTH, depth=self.MEMDEPTH)
        writeport = self.mem.get_port(write_capable=True, mode = READ_FIRST)
        readport = self.mem.get_port(has_re=True)
        self.specials += writeport, readport
        self.ios = {writeport.adr, writeport.dat_w, writeport.we, readport.dat_r, readport.adr, readport.re}
        self.submodules.memory = FSM(reset_state = "RESET")
        readbit = Signal(max = self.MEMWIDTH)
        self.writebyte = Signal(max=self.MEMDEPTH)
        written = Signal()
        dat_r_temp = Signal(max= self.MEMWIDTH)
        self.memory.act("RESET",
                NextValue(written, 0),
                NextValue(readport.re, 1),
                NextValue(readport.adr, 0),
                NextValue(writeport.adr, 0),
                NextState("IDLE")
        )
        # this state is useless
        self.memory.act("IDLE",
            NextState("IDLE")
        )
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
        #TODO: NOT CHECKED!!
        # memory full
        self.comb += self.error[0].eq((writeport.adr==readport.adr)&(written==1))
        # Custom Receiver
        self.submodules.receiver = FSM(reset_state = "IDLE")
        self.receiver.act("IDLE",
                #NOTE: simplify with cat
                NextValue(spislave.miso[0:5], self.error),
                NextValue(spislave.miso[5:], self.laserfsmstate),
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
                    If((writeport.adr==readport.adr)&(written==1),
                        NextValue(command, self.COMMANDS.RECVCOMMAND)
                    ).
                    Else(
                        NextValue(command, self.COMMANDS.WRITE_L)
                    )
                )
                # Else; Command invalid or memory full nothing happens
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
        #TODO: je hebt hier over bytes --> volgens mij moet je het maal 8 doen
        bitcounter = Signal(max=int(self.VARIABLES['SCANLINE_DATA_SIZE']))
        facetcounter = Signal(max=int(self.VARIABLES['FACETS']))
        # stable counter used for both spinup and photo diode stable
        spinupticks = int(self.VARIABLES['SPINUP_TIME']*self.VARIABLES['CRYSTAL_HZ'])
        stableticks = int(self.VARIABLES['STABLE_TIME']*self.VARIABLES['CRYSTAL_HZ'])
        stablecounter = Signal(max=max(spinupticks, stableticks))
        ticksinfacet = self.VARIABLES['CRYSTAL_HZ']/(self.VARIABLES['RPM']*60*self.VARIABLES['FACETS'])
        self.tickcounter = Signal(max=int(ticksinfacet*10))
        #tickcounter = Signal()
        #counter = Signal(max=self.MAXPERIOD.bit_length())
        self.submodules.laserfsm = FSM(reset_state = "STOP")
        self.laserfsm.act("STOP",
            NextValue(stablecounter, 0),
            NextValue(bitcounter, 0),
            NextValue(laser0, 0),
            NextValue(poly_en, 1),
            NextValue(self.error[0], 0), # there is no read error 
            NextValue(readbit,0),
            If(self.laserfsmstate==self.STATES.START,
                 NextValue(poly_en, 0),
                 NextState("SPINUP")
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
        self.laserfsm.act("STATE_WAIT_STABLE",
            NextValue(laser0, 1),
            NextValue(self.tickcounter, self.tickcounter+1),
            NextValue(stablecounter, stablecounter+1),
            If(photodiode_fall, 
               NextValue(self.tickcounter, 0),
               If((self.tickcounter>int(ticksinfacet*(1-self.VARIABLES['JITTER_THRESH'])))&
                  (self.tickcounter<int(ticksinfacet*(1+self.VARIABLES['JITTER_THRESH']))),
                  NextState('WAIT_FOR_DATA_RUN')
               )
            ),
            If(stablecounter>stableticks-1,
               NextValue(self.error[2], 1),
               NextState('STOP')
            ),
            If(self.laserfsmstate!=self.STATES.START,
                 NextState("STOP")
            )  
        )
        self.laserfsm.act('WAIT_FOR_DATA_RUN',
            NextValue(laser0, 0),
            NextValue(self.tickcounter, self.tickcounter+1),
            If(self.tickcounter>int(self.VARIABLES['TICKS_START']),
                #TODO: replace with data run and fix with going back to wait stable
                NextState('START')
            ) 
        )
        self.laserfsm.act("START",
            If(bitcounter >= self.VARIABLES['SCANLINE_DATA_SIZE']-1,
               NextValue(bitcounter, 0),
               # if there is no data, led off and report error
               #NOTE: would also make sense to report error if not been written yet and you try to read
               If(written==0,
                    NextState("READ"), # you nead to read again, wrong value
                    NextValue(readport.re, 0),
                    NextValue(laser0, 0),
                    NextValue(self.error[0], 1)
               ).
               Else(
                    NextValue(self.error[0], 0),
                    NextValue(dat_r_temp, dat_r_temp>>1),
                    NextValue(laser0, dat_r_temp[0]),
                    NextValue(readbit, readbit+1),
                    # you need to read again!
                    # move to next addres if end is reached
                    If(readbit==self.MEMWIDTH-1,
                        NextState("READ"), # you nead to read again, wrong value
                        NextValue(readport.re, 0),
                        NextValue(readport.adr, readport.adr+1),
                        If(readport.adr+1==writeport.adr,
                            NextValue(written,0)
                        ).
                        #NOTE: count wrap around
                        Elif((readport.adr+1==self.MEMDEPTH)&(writeport.adr==0),
                            NextValue(written,0)
                        )
                )
               )
            ).
            Else(
                NextValue(bitcounter, bitcounter+1)
            ),
            If(self.laserfsmstate!=self.STATES.START,
               NextState("STOP")
            )
        )
        self.laserfsm.act("READ",
            NextValue(bitcounter, bitcounter+1),
            NextValue(readport.re, 1),
            NextValue(dat_r_temp, readport.dat_r),
            NextState("START")
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