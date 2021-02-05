# use abstraction already present in luna !
# from collections import namedtuple

# from nmigen import Signal, Cat, Elaboratable
# from nmigen import Memory, Module
# from litex.soc.cores.spi import SPISlave

# # NOTE: following doesnt work due to bug in pylint https://github.com/PyCQA/pylint/issues/3876
# # def customnamedtuple(typename, field_names) -> namedtuple:
# #    return namedtuple(typename, field_names,
# #                      defaults=range(len(field_names)))


# STATES = namedtuple('STATES', ['START', 'STOP'], defaults=range(2))()
# COMMANDS = namedtuple('COMMANDS', ['EMPTY', 'FILLED', 'EXIT', 'ABORT'],
#                       defaults=range(4))()
# ERRORS = namedtuple('ERRORS', ['MEMFULL', 'MEMREAD', 'INVALID'],
#                     defaults=range(3))()
# CHUNKSIZE = 15  # first chunk is command
# # one block is 4K bits, there are 32 blocks (officially 20 in HX4K)
# MEMWIDTH = 16
# MEMDEPTH = 256
# VARIABLES = {'CRYSTAL_HZ': 50E6}


# class SPIParser(Elaboratable):
#     '''parses instructions by beagleg and stores them in memory
#      unless memory is full'''

#     def __init__(self):
#         # step generator connections
#         self.written = Signal()
#         self.error = Signal()
#         self.fsmstate = Signal()
#         # memory connections
#         self.read_adr = Signal(range(MEMDEPTH))
#         self.write_adr = Signal(range(MEMDEPTH))
#         self.write_data = Signal(range(MEMWIDTH))
#         self.write_en = Signal()

#     def elaborate(self, platform):
#         m = Module()
#         # Memfull trigger
#         # A forward and backward lagged value with resprect to current
#         # memory position to trigger memory full.
#         dummyf = Signal(range(2*MEMDEPTH))
#         dummyb = Signal(range(-MEMDEPTH, MEMDEPTH))
#         m.d.sync += dummyf.eq(self.write_adr+CHUNKSIZE+1)
#         m.d.sync += dummyb.eq(self.write_adr+CHUNKSIZE+1-(MEMDEPTH-1))
#         # memfull is calculated prior to transaction to prevent
#         # conflict
#         memfulld = Signal()

#         command = Signal()
#         chunkcnt = Signal(range(CHUNKSIZE+1))
#         # spi
#         spislave = SPISlave(platform.request("spi"),
#                             data_width=MEMWIDTH)
#         m.submodules.slave = spislave
#         # Done detector
#         done_d = Signal()
#         done_rise = Signal()
#         m.d.sync += done_d.eq(spislave.done)
#         m.d.comb += done_rise.eq(spislave.done & ~done_d)
#         # Start detector
#         start_d = Signal()
#         start_rise = Signal()
#         m.d.sync += start_d.eq(spislave.start)
#         m.d.comb += start_rise.eq(spislave.start & ~start_d)
#         # backup of spi_mosi
#         spi_mosi = Signal(MEMWIDTH)
#         with m.FSM(reset='RESET', name='parser'):
#             with m.State('RESET'):
#                 m.next = 'WAITFORSTART'
#                 m.d.sync += [command.eq(1),
#                              chunkcnt.eq(0)]
#             with m.State('WAITFORSTART'):
#                 with m.If(start_rise):
#                     m.next = 'WAITFORDONE'
#                 with m.Else():
#                     with m.If(self.written):
#                         with m.If((dummyf > self.read_adr) &
#                                   (self.read_adr > self.write_adr)):
#                             self.error[ERRORS.MEMFULL].eq(1)
#                         with m.Elif((dummyb > self.read_adr) &
#                                     (self.read_adr < self.write_adr)):
#                             self.error[ERRORS.MEMFULL].eq(1)
#                         with m.Else():
#                             self.error[ERRORS.MEMFULL].eq(0)
#                     with m.Else():
#                         self.error[ERRORS.MEMFULL].eq(0)
#                         spislave.miso.eq(Cat([self.error,
#                                               self.fsmstate]))
#                         memfulld.eq(self.error[ERRORS.MEMFULL])
#             with m.State('WAITFORDONE'):
#                 with m.If(done_rise):
#                     # backup mosi in case it changes
#                     spi_mosi.eq(spislave.mosi)
#                     # controller is notified peripheral is busy (ugly hack)
#                     spislave.miso.eq(4)
#                     with m.If(command):
#                         m.next = 'PROCESSCOMMAND'
#                     with m.Else():
#                         with m.If(chunkcnt >= CHUNKSIZE-1):
#                             chunkcnt.eq(0)
#                             command.eq(1)
#                         with m.Else():
#                             chunkcnt.eq(chunkcnt+1)
#                         command.eq(1)
#                         self.write_data.eq(spislave.mosi)
#                         self.write_en.eq(1)
#                         m.next = 'WRITE'
#             # command used as extra parameter this is confusing
#             with m.State('PROCESSCOMMAND'):
#                 m.next = 'WAITFORSTART'
#                 with m.If(spi_mosi == COMMANDS.EMPTY):
#                     command.eq(0)
#                 with m.Elif(spi_mosi == COMMANDS.FILLED):
#                     command.eq(1)
#                     self.fsmstate.eq(STATES.START)
#                 with m.Elif(spi_mosi == COMMANDS.EXIT):
#                     command.eq(0)
#                     self.fsmstate.eq(STATES.STOP)
#                 with m.Elif(spi_mosi == COMMANDS.ABORT):
#                     pass
#                 with m.Else():
#                     self.error[ERRORS.INVALID].eq(1)
#                     command.eq(0)
#             with m.State('WRITE'):
#                 m.next = 'WAITFORSTART'
#                 self.written.eq(1)
#                 with m.If((self.write_adr + 1) == MEMDEPTH):
#                     self.write_adr.eq(0)
#                 with m.Else():
#                     self.write_adr.eq(self.write_adr+1)
#                     self.write_en.eq(0)


# class GPARSER(Elaboratable):
#     '''
#     FPGA core to parse instructions provided by beagleG
#     '''
#     def __init__(self):
#         self.mem = Memory(width=MEMWIDTH, depth=MEMDEPTH)

#     # NOTE: if platform is None interpret as test
#     def elaborate(self, platform):
#         # core GPARSER module
#         m = Module()
#         m.submodules.rdport = rdport = self.mem.read_port()
#         m.submodules.wrport = wrport = self.mem.write_port()
#         # spi parser connections to core
#         m.submodules.spiparser = spiparser = SPIParser()
#         m.d.comb += [
#             spiparser.read_adr.eq(rdport.addr),
#             spiparser.write_en.eq(wrport.en),
#             spiparser.write_adr.eq(wrport.addr),
#             spiparser.write_data.eq(wrport.data)
#         ]
#         # NOTE: written, error and fsm state are not connected

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