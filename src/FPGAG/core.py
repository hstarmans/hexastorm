from collections import namedtuple

from nmigen import Signal, If, Else, Elif, Cat, Elaboratable
from nmigen import ClockDomain, Memory, Module
from litex.soc.cores.clock import iCE40PLL
from litex.soc.cores.spi import SPISlave



class SPIParser(Elaboratable):
    'parses instructions by beagleg and stores them in memory'
    def elaborate(self, platform, test=False):
        m.module()
        with m.FSM(reset='RESET', name='parser'):
            with m.State('RESET'):
                m.next = 'WAITFORSTART'
                m.d.sync += [command.eq(1),
                             chunkcnt.eq(0)]
            with m.State('WAITFORSTART'):
                with m.If(start_rise):
                    m.next = 'WAITFORDONE'
                with m.Else():
                    with m.If(written):
                        with m.If((dummyf > rdport.addr) &
                                    (rdport.addr > wrport.addr)):
                            self.error[self.ERRORS.MEMFULL].eq(1)
                        with m.Elif((dummyb > rdport.addr) &
                                    (rdport.addr < wrport.addr)):
                            self.error[self.ERRORS.MEMFULL].eq(1)
                        with m.Else():
                            self.error[self.ERRORS.MEMFULL].eq(0)
                    with m.Else():
                        self.error[self.ERRORS.MEMFULL].eq(0)
                        spislave.miso.eq(Cat([self.error,
                                                self.fsmstate]))
                        memfulld.eq(self.error[self.ERRORS.MEMFULL])
            with m.State('WAITFORDONE'):
                # backup of mosi in case it changes
                spi_mosi.eq(spislave.mosi)
                with m.If(done_rise):
                    # controller is notified peripheral is busy (ugly hack)
                    spislave.miso.eq(4)
                    with m.If(command):
                        m.next = 'PROCESSCOMMAND'
                    with m.Else():
                        with m.If(chunkcnt >= self.CHUNKSIZE-1):
                            chunkcnt.eq(0)
                            command.eq(1)
                        with m.Else():
                            chunkcnt.eq(chunkcnt+1)
                        command.eq(1)
                        self.dat_w.eq(spislave.mosi)
                        self.we.eq(1)
                        m.next = 'WRITE'
            # command used as extra parameter this is confusing
            with m.State('PROCESSCOMMAND'):
                m.next = 'WAITFORSTART'
                with m.If(spi_mosi == self.COMMANDS.EMPTY):
                    command.eq(0)
                with m.Elif(spi_mosi == self.COMMANDS.FILLED):
                    command.eq(1)
                    self.fsmstate.eq(self.STATES.START)
                with m.Elif(spi_mosi == self.COMMANDS.EXIT):
                    command.eq(0)
                    self.fsmstate.eq(self.STATES.STOP)
                with m.Elif(spi_mosi == self.COMMANDS.ABORT):
                    pass
                with m.Else():
                    self.error[self.ERRORS.INVALID].eq(1)
                    command.eq(0)
            with m.State('WRITE'):
                m.next = 'WAITFORSTART'
                written.eq(1)
                with m.If((self.adr + 1) == self.MEMDEPTH):
                    self.adr.eq(0)
                with m.Else():
                    self.adr.eq(self.adr+1)
                    self.we.eq(0)





class GPARSER(Elaboratable):
    '''
    FPGA core to parse instructions provided by beagleG
    '''
    @staticmethod
    def customnamedtuple(names):
        states = namedtuple('States', names,
                            defaults=tuple(range(len(names))))
        return states()
    STATES = customnamedtuple.__func__(('STOP', 'START'))
    COMMANDS = customnamedtuple.__func__(('EMPTY', 'FILLED', 'EXIT', 'ABORT'))
    ERRORS = customnamedtuple.__func__(('MEMFULL', 'MEMREAD',
                                        'NOTSTABLE', 'INVALID', 'INVALIDLINE'))
    CHUNKSIZE = 15  # first chunk is command
    # one block is 4K bits, there are 32 blocks (officially 20 in HX4K)
    MEMWIDTH = 8
    MEMDEPTH = 512

    VARIABLES = {'CRYSTAL_HZ': 50E6}

    def __init__(self):
        # TODO: THIS IS WRONG YOU HAVE DUAL ADDRESS!
        self.adr = Signal(4)
        self.dat_r = Signal(8)
        self.dat_w = Signal(8)
        self.we = Signal()
        self.re = Signal()
        self.mem = Memory(width=self.MEMWIDTH, depth=self.MEMDEPTH)

    def elaborate(self, platform, test=False):
        m = Module()
        clk100 = platform.request('clk100')
        m.domains.cd_sys = ClockDomain(reset_less=True)
        platform.add_period_constraint(m.domains.cd_sys.clk, 20)
        if not test:
            m.submodules.pll = pll = iCE40PLL()
            pll.register_clkin(clk100, 100e6)
            pll.create_clkout(m.domains.cd_sys, self.VARIABLES['CRYSTAL_HZ'])
        # three submodules; SPI receiver, memory and laser state machine
        # full byte state
        self.fsmstate = Signal(3)    # state laser module 5-8 bit
        self.error = Signal(5)       # error              0-5 bit
        # Memory element
        # Rules:
        #        read/write address cannot be equal
        # readbit, current bit read
        # written to detect if already information is written to memory
        #         equals zero if memory is complety read or empty
        # dat_r_temp , data is shifted after read. Done on different element
        # as it is believed this is not possible
        # sram memory is 32 blocks... each block has its own ports
        # one block 8*512 = 4096 bits currently used
        m.submodules.rdport = rdport = self.mem.read_port()
        m.submodules.wrport = wrport = self.mem.write_port()
        # TODO: migen also has  a combinatorial here !
        readbit = Signal(range(self.MEMWIDTH))
        written = Signal()
        self.dat_r_new = Signal(self.MEMWIDTH)
        dat_r_old = Signal(self.MEMWIDTH)
        # Receiver State Machine
        # TODO: tried to paralize receiver and parser but this gives errors
        #       I am not able achieve agreement between Linux host and FPGA
        #       on memfull
        #       Memfull is not correctly propagated every 10K operations
        #       which results in failure.
        # Consists out of component from litex and own custom component
        # Detects whether new command is available
        self.spi = platform.request("spi")
        spislave = SPISlave(self.spi, data_width=8)
        m.submodules.slave = spislave
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
        dummyf = Signal(range(2*self.MEMDEPTH))
        dummyb = Signal(range(-self.MEMDEPTH, max=self.MEMDEPTH))
        self.sync += dummyf.eq(wrport.addr+self.CHUNKSIZE+1)
        self.sync += dummyb.eq(wrport.addr+self.CHUNKSIZE+1-(self.MEMDEPTH-1))
        spi_mosi = Signal(self.MEMWIDTH)
        memfulld = Signal()
        command = Signal()
        chunkcnt = Signal(range(self.CHUNKSIZE+1))


####################

        # # the original motor driver was designed for 6 facets and pulsed for eached facet
        # polyperiod = int(self.VARIABLES['CRYSTAL_HZ']/(self.VARIABLES['RPM']/60)/(6*2))
        # pwmcounter = Signal(max=polyperiod)
        # self.poly_pwm = platform.request("poly_pwm")
        # self.sync += If(pwmcounter == 0,
        #         self.poly_pwm.eq(~self.poly_pwm),
        #         pwmcounter.eq(polyperiod-1)).Else(
        #         pwmcounter.eq(pwmcounter - 1)
        #         )
        # # Laser FSM
        # # Laser FSM controls the laser, polygon anld output to motor
        # self.facetcnt = Signal(max=self.VARIABLES['FACETS'])
        # # stable counter used for both spinup and photo diode stable
        # spinupticks = round(self.VARIABLES['SPINUP_TIME']*self.VARIABLES['CRYSTAL_HZ'])
        # stableticks = round(self.VARIABLES['STABLE_TIME']*self.VARIABLES['CRYSTAL_HZ'])
        # stablecounter = Signal(max=max(spinupticks, stableticks))   # counter is used twice, hence the max
        # stablethresh = Signal(max=stableticks)
        # self.lasercnt = Signal(max=LASERTICKS)
        # self.scanbit = Signal(max=self.BITSINSCANLINE+1)
        # self.tickcounter = Signal(max=int(self.ticksinfacet*2))
        # self.submodules.laserfsm = FSM(reset_state = "RESET")
        # # leesfoutdetectie:
        # #   in het begin zijn lees en schrijfadres gelijk
        # #   als er geschreven is dan is het schrijf adres een groter dan lees adres
        # #   als er geschreven is en het volgende adres waarvan je gaat lezen nog niet beschreven is --> lees fout
        # # op het moment kost lezen een tick, dit zou voorkomen kunnen worden
        # # tick counter; number of ticks in a facet for the oscillator
        # # laser counter; laser operates at reduced speed this controlled by this counter
        # # readbit counter; current bit position in memory
        # # scanbit counter; current bit position along scanline
        # readtrig = Signal()
        # self.submodules.readmem= FSM(reset_state = "RESET")
        # self.readmem.act("RESET",
        #     NextValue(self.readport.adr, 0),
        #     NextValue(self.readport.re, 1),
        #     NextValue(written, 0),
        #     NextState("WAIT")
        # )
        # self.readmem.act("WAIT",
        #     If(readtrig,
        #        NextValue(self.readport.re, 0),
        #        NextState("READ")
        #     )
        # )
        # self.readmem.act("READ",
        #     NextValue(readtrig, 0),
        #     NextValue(self.readport.re, 1),
        #     If((self.readport.adr == writeport.adr)&(written == 0),
        #        NextValue(self.error[self.ERRORS.MEMREAD], 1),
        #     ).
        #     Else(
        #         #TODO: you could split into two signals --> one if ever memread occured, other memread signal
        #         NextValue(self.error[self.ERRORS.MEMREAD], 0),
        #         NextValue(self.dat_r_new, self.readport.dat_r)
        #     ),
        #     # always increase address, if you move over the write set written to zero
        #     If(self.readport.adr+1==self.MEMDEPTH,
        #         NextValue(self.readport.adr, 0),
        #         If((writeport.adr == 0)&(self.VARIABLES['SINGLE_LINE']==False), NextValue(written,0))
        #     ).
        #     Else(
        #         NextValue(self.readport.adr, self.readport.adr+1),
        #         If((self.readport.adr+1 == writeport.adr)&(self.VARIABLES['SINGLE_LINE']==False), NextValue(written,0))
        #     ),
        #     NextState("WAIT")
        # )
        # self.laserfsm.act("RESET",
        #     NextValue(self.readport.adr, 0),
        #     NextValue(writeport.adr, 0),
        #     NextState("STOP")
        # )
        # self.laser0 = platform.request("laser0")
        # self.poly_en = platform.request("poly_en")
        # self.photodiode = platform.request("photodiode")
        # self.laserfsm.act("STOP",
        #     NextValue(stablethresh, stableticks-1),
        #     NextValue(stablecounter, 0),
        #     NextValue(self.facetcnt, 0),
        #     NextValue(self.tickcounter, 0),
        #     NextValue(self.scanbit, 0),
        #     NextValue(self.lasercnt, 0),
        #     NextValue(self.laser0, 0),
        #     NextValue(self.poly_en, 1),
        #     NextValue(readbit,0),
        #     If(self.fsmstate==self.STATES.START,
        #         If(self.photodiode == 0,
        #             NextValue(self.fsmstate, self.STATES.STOP),
        #             NextState("STOP")
        #         ).
        #         Else(
        #             NextValue(readtrig, 1),
        #             NextValue(self.error[self.ERRORS.NOTSTABLE], 0),
        #             NextValue(self.error[self.ERRORS.MEMREAD], 0),
        #             NextValue(self.poly_en, 0),
        #             NextState("SPINUP")
        #         )
        #     ).
        #     Elif(self.fsmstate==self.STATES.MOTORTEST,
        #         NextValue(self.poly_en, 0),
        #         NextState("MOTORTEST")
        #     ).
        #     Elif(self.fsmstate==self.STATES.LASERTEST,
        #         NextValue(self.laser0, 1),
        #         NextState("LASERTEST")
        #     ).
        #     Elif(self.fsmstate==self.STATES.LINETEST,
        #         NextValue(self.laser0, 1),
        #         NextValue(self.poly_en, 0),
        #         NextState("LINETEST")
        #     ).
        #     Elif(self.fsmstate==self.STATES.PHOTODIODETEST,
        #         # photodiode should be high with laser off
        #         # something is wrong, this makes sure error is produced
        #         If(self.photodiode == 0,
        #             NextValue(self.laser0, 1),
        #             NextValue(self.poly_en, 1)
        #         ).
        #         Else(
        #             NextValue(self.laser0, 1),
        #             NextValue(self.poly_en, 0),
        #         ),
        #         NextState("PHOTODIODETEST")
        #     )
        # )
        # self.laserfsm.act("MOTORTEST",
        #     If(self.fsmstate!=self.STATES.MOTORTEST,
        #          NextState("STOP")
        #     )
        # )
        # self.laserfsm.act("LASERTEST",
        #     If(self.fsmstate!=self.STATES.LASERTEST,
        #          NextState("STOP")
        #     )
        # )
        # self.laserfsm.act("LINETEST",
        #     If(self.fsmstate!=self.STATES.LINETEST,
        #          NextState("STOP")
        #     )
        # )
        # # Photodiode rising edge detector
        # photodiode_d = Signal()
        # self.laserfsm.act("PHOTODIODETEST",
        #     If((self.photodiode == 0) & (self.poly_en == 0),
        #         NextValue(self.fsmstate, self.STATES.STOP),
        #         NextState("STOP")
        #     ).
        #     Elif(self.fsmstate!=self.STATES.PHOTODIODETEST,
        #          NextState("STOP")
        #     )
        # )
        # self.laserfsm.act("SPINUP",
        #     NextValue(stablecounter, stablecounter + 1),
        #     If(stablecounter>spinupticks-1,
        #         NextState("STATE_WAIT_STABLE"),
        #         NextValue(self.laser0, 1),
        #         NextValue(stablecounter, 0),
        #     )
        # )
        # self.laserfsm.act("STATE_WAIT_STABLE",
        #     NextValue(stablecounter, stablecounter+1),
        #     NextValue(photodiode_d, self.photodiode),
        #     If(stablecounter>=stablethresh,
        #        NextValue(self.error[self.ERRORS.NOTSTABLE], 1),
        #        NextValue(self.fsmstate, self.STATES.STOP),
        #        NextState('RESET')
        #     ).
        #     Elif(~self.photodiode&~photodiode_d,
        #        NextValue(self.tickcounter, 0),
        #        NextValue(self.laser0, 0),
        #        If((self.tickcounter>self.ticksinfacet-self.JITTERTICKS)&
        #           (self.tickcounter<self.ticksinfacet+self.JITTERTICKS),
        #           If(self.facetcnt==self.VARIABLES['FACETS']-1, NextValue(self.facetcnt, 0)).
        #           Else(NextValue(self.facetcnt, self.facetcnt+1)),
        #           NextValue(stablecounter, 0),
        #           If((self.VARIABLES['SINGLE_FACET']==True)&(self.facetcnt>0),
        #             NextState('WAIT_END')
        #           ).
        #           Else(
        #             NextValue(stablethresh, min(round(10.1*self.ticksinfacet), stableticks)),  #TODO: lower!
        #             NextState('READ_INSTRUCTION')
        #           )
        #        ).
        #        Else(
        #            NextState('WAIT_END')
        #        )
        #     ).
        #     Elif(self.fsmstate!=self.STATES.START,
        #          NextState("RESET")
        #     ).  
        #     Else(
        #         NextValue(self.tickcounter, self.tickcounter+1)
        #     )
        # )
        # self.laserfsm.act('READ_INSTRUCTION',
        #     NextValue(self.tickcounter, self.tickcounter+1),
        #     If(readtrig == 0,
        #        If(self.error[self.ERRORS.MEMREAD] == 1,
        #            # move back the address and read again
        #            If(self.readport.adr == 0,
        #                 NextValue(self.readport.adr, self.MEMDEPTH-1)
        #            ).
        #            Else(
        #                NextValue(self.readport.adr, self.readport.adr-1)
        #            ),
        #            NextValue(readtrig, 1),
        #            NextState("WAIT_END")
        #        ).
        #        Elif(self.dat_r_new == self.INSTRUCTIONS.STOP,
        #            NextState("RESET"),
        #            NextValue(self.fsmstate, self.STATES.STOP)
        #        ).
        #        Elif(self.dat_r_new == self.INSTRUCTIONS.SCAN,
        #            NextState('WAIT_FOR_DATA_RUN'),
        #            NextValue(readtrig, 1),
        #        ).
        #        Else(NextState("RESET"),
        #             NextValue(self.fsmstate, self.STATES.STOP),
        #             NextValue(self.error[self.ERRORS.INVALIDLINE], 1),
        #        )
        #     )
        # )
        # self.laserfsm.act('WAIT_FOR_DATA_RUN',
        #     NextValue(self.tickcounter, self.tickcounter + 1),
        #     If(readtrig == 0,
        #         If(self.error[self.ERRORS.MEMREAD] == 1,
        #             NextValue(dat_r_old, 0)
        #             ).
        #         Else(
        #             NextValue(dat_r_old, self.dat_r_new)
        #         ),
        #         NextValue(readbit, 0),
        #         NextValue(self.scanbit, 0),
        #         NextValue(self.lasercnt, 0),
        #         If(self.tickcounter==int(self.VARIABLES['START%']*self.ticksinfacet-2),
        #             NextState('DATA_RUN')
        #         ).
        #         Elif(self.tickcounter>int(self.VARIABLES['START%']*self.ticksinfacet-2),
        #             #NextValue(self.error[self.ERRORS.INVALID], 1),   #TODO: replace with timeout
        #             NextState('DATA_RUN')
        #         )
        #     )
        # )
        # self.laserfsm.act("DATA_RUN",
        #     NextValue(self.tickcounter, self.tickcounter + 1),
        #     If(self.lasercnt == 0,
        #         #NOTE: readbit and scanbit counters can be different
        #         #      readbit is your current position in memory and scanbit your current byte position in scanline
        #         If(self.scanbit >= self.BITSINSCANLINE,
        #             NextState("WAIT_END")
        #         ).
        #         Else(
        #             NextValue(self.lasercnt, LASERTICKS-1),
        #             NextValue(self.scanbit, self.scanbit+1),
        #             # read from memory before the spinup
        #             # it is triggered here again, so fresh data is available once the end is reached
        #             # if read bit is 0, trigger a read out unless the next byte is outside of line
        #             If(readbit == 0,
        #                 NextValue(readtrig, 1),
        #                 NextValue(readbit, readbit+1),
        #                 NextValue(dat_r_old, dat_r_old>>1)
        #             ).
        #             # final read bit copy memory
        #             # move to next address, i.e. byte, if end is reached
        #             Elif(readbit==self.MEMWIDTH-1,
        #                 If(self.error[self.ERRORS.MEMREAD] == 1,
        #                     NextValue(dat_r_old, 0)
        #                 ).
        #                 Else(
        #                     NextValue(dat_r_old, self.dat_r_new)
        #                 ),
        #                 NextValue(readbit, 0)
        #             ).
        #             Else(
        #                 NextValue(readbit, readbit+1),
        #                 NextValue(dat_r_old, dat_r_old>>1)
        #             ),
        #             NextValue(self.laser0, dat_r_old[0])
        #         )
        #     ).
        #     Else(
        #         NextValue(self.lasercnt, self.lasercnt - 1)
        #     )
        # )
        # self.laserfsm.act("WAIT_END",
        #     NextValue(stablecounter, stablecounter+1),
        #     NextValue(self.tickcounter, self.tickcounter+1),
        #     If(self.tickcounter>=round(self.ticksinfacet-self.JITTERTICKS-1),
        #        NextState("STATE_WAIT_STABLE"),
        #        NextValue(self.laser0, 1),
        #     )
        # )