from amaranth import Elaboratable, Module, Signal
from luna.gateware.memory import TransactionalizedFIFO

from .constants import INSTRUCTIONS, MEMWIDTH, params


class Laserhead(Elaboratable):
    """Controller of laser scanner with rotating mirror or prism

    I/O signals:
    O: synchronized   -- if true, laser is in sync and prism is rotating
    I: synchronize    -- activate synchronization
    I: singlefacet    -- activate single facet
    I: expose_start   -- start reading lines and exposing
    O: expose_finish  -- exposure is finished
    O: error          -- error signal
    O: lasers         -- laser pin
    O: pwm            -- pulse for scanner motor
    O: enable_prism   -- enable pin scanner motor
    I: photodiode     -- trigger for photodiode
    O: photodiode_t   -- high if photodiode triggered in this cycle
    O: photodiode_2   -- an attemp to add laser microscopy (not tested)
    O: read_commit    -- finalize read transactionalizedfifo
    O: read_en        -- enable read transactionalizedfifo
    I: read_data      -- read data from transactionalizedfifo, AKA busy
    I: empty          -- signal wether fifo is empty
    O: step           -- step signal
    O: direction      -- direction signal
    """

    def __init__(self, platform):
        """
        platform   -- pass test platform
        """
        self.platform = platform
        self.dct = params(platform)
        self.status = Signal()
        self.lasers = Signal(2)
        self.pwm = Signal()
        self.enable_prism = Signal()
        self.synchronize = Signal()
        self.synchronized = Signal()
        self.singlefacet = Signal()
        self.error = Signal()
        self.ticksinfacet = Signal(range(self.dct["TICKSINFACET"] * 2))
        self.photodiode = Signal()
        self.photodiode_t = Signal()
        self.photodiode_2 = Signal()
        # memory; read operations
        self.read_commit = Signal()
        self.read_en = Signal()
        self.read_data = Signal(MEMWIDTH)
        self.read_discard = Signal()
        self.empty = Signal()
        # memory; write operations
        self.write_commit_2 = Signal()
        self.write_en_2 = Signal()
        self.write_data_2 = Signal(MEMWIDTH)
        self.write_discard_2 = Signal()
        self.full = Signal()

        self.expose_finished = Signal()
        self.expose_start = Signal()
        self.step = Signal()
        self.dir = Signal()
        
        self.process_lines = Signal()

    def elaborate(self, platform):
        m = Module()
        dct = self.dct

        # Pulse generator for prism motor
        pwmcnt = Signal(range(dct["POLYPERIOD"]))
        # photodiode_triggered
        # TODO: not added to tests
        photodiodecnt = Signal(range(dct["TICKSINFACET"] * 2))
        triggered = Signal()
        with m.If(photodiodecnt < (dct["TICKSINFACET"] * 2 - 1)):
            with m.If(~self.photodiode):
                m.d.sync += triggered.eq(1)
            m.d.sync += photodiodecnt.eq(photodiodecnt + 1)
        with m.Else():
            m.d.sync += [
                self.photodiode_t.eq(triggered),
                photodiodecnt.eq(0),
                triggered.eq(0),
            ]

        # step generator, i.e. slowest speed is 1/(2^4-1)
        stephalfperiod = Signal(dct["BITSINSCANLINE"].bit_length() + 4)
        stepcnt = Signal.like(stephalfperiod)

        # TODO: this can be removed with the new driver!
        # pwm is always created but can be deactivated
        with m.If(pwmcnt == 0):
            m.d.sync += [
                self.pwm.eq(~self.pwm),
                pwmcnt.eq(dct["POLYPERIOD"] - 1),
            ]
        with m.Else():
            m.d.sync += pwmcnt.eq(pwmcnt - 1)

        # Laser FSM
        # stable thresh is changed, larger at start and then lowered
        stablethresh = Signal(range(dct["STABLETICKS"]))
        facetcnt = Signal(range(dct["FACETS"]))
        lasercnt = Signal(range(dct["LASERTICKS"]))
        scanbit = Signal(range(dct["BITSINSCANLINE"] + 1))
        tickcounter = Signal(
            range(max(dct["SPINUPTICKS"], dct["STABLETICKS"]))
        )
        scanlinenumber = Signal(range(255))
        photodiode = self.photodiode
        read_data = self.read_data
        write_data_2 = self.write_data_2
        write_new = Signal.like(write_data_2)
        read_old = Signal.like(read_data)
        readbit = Signal(range(MEMWIDTH))
        photodiode_d = Signal()
        lasers = self.lasers
        if self.platform.name == "Test":
            self.stephalfperiod = stephalfperiod
            self.tickcounter = tickcounter
            self.scanbit = scanbit
            self.lasercnt = lasercnt
            self.facetcnt = facetcnt

        # Exposure start detector
        expose_start_d = Signal()


        m.d.sync += expose_start_d.eq(self.expose_start)
        with m.If((expose_start_d == 0) & self.expose_start):
            m.d.sync += [self.process_lines.eq(1), self.expose_finished.eq(0)]

        with m.FSM(init="RESET") as laserfsm:
            with m.State("RESET"):
                m.d.sync += [self.error.eq(0), self.ticksinfacet.eq(0)]
                m.next = "STOP"
            with m.State("STOP"):
                m.d.sync += [
                    stablethresh.eq(dct["STABLETICKS"] - 1),
                    tickcounter.eq(0),
                    self.synchronized.eq(0),
                    self.enable_prism.eq(0),
                    readbit.eq(0),
                    facetcnt.eq(0),
                    scanbit.eq(0),
                    lasercnt.eq(0),
                    lasers.eq(0),
                ]
                with m.If(self.synchronize & (~self.error)):
                    # laser is off, photodiode cannot be triggered
                    with m.If(self.photodiode == 0):
                        m.d.sync += self.error.eq(1)
                        m.next = "STOP"
                    with m.Else():
                        m.d.sync += [self.error.eq(0), self.enable_prism.eq(1)]
                        m.next = "SPINUP"
            with m.State("SPINUP"):
                with m.If(tickcounter > dct["SPINUPTICKS"] - 1):
                    # turn on single channel
                    m.d.sync += [
                        self.lasers.eq(int("10", 2)),
                        tickcounter.eq(0),
                    ]
                    m.next = "WAIT_STABLE"
                with m.Else():
                    m.d.sync += tickcounter.eq(tickcounter + 1)
            with m.State("WAIT_STABLE"):
                m.d.sync += photodiode_d.eq(photodiode)
                with m.If(tickcounter >= stablethresh):
                    m.d.sync += self.error.eq(1)
                    m.next = "STOP"
                with m.Elif(~photodiode & ~photodiode_d):
                    m.d.sync += [tickcounter.eq(0), lasers.eq(0)]
                    with m.If(
                        (
                            tickcounter
                            > (dct["TICKSINFACET"] - 1) - dct["JITTERTICKS"]
                        )
                    ):
                        m.d.sync += [
                            self.synchronized.eq(1),
                            self.ticksinfacet.eq(tickcounter),
                        ]
                        with m.If(facetcnt == dct["FACETS"] - 1):
                            m.d.sync += facetcnt.eq(0)
                        with m.Else():
                            m.d.sync += facetcnt.eq(facetcnt + 1)
                        with m.If(self.singlefacet & (facetcnt > 0)):
                            m.next = "WAIT_END"
                        with m.Elif(self.empty | ~self.process_lines):
                            m.next = "WAIT_END"
                        with m.Else():
                            # TODO: 10 is too high, should be lower
                            thresh = min(
                                round(10.1 * dct["TICKSINFACET"]),
                                dct["STABLETICKS"],
                            )
                            m.d.sync += [
                                stablethresh.eq(thresh),
                                self.read_en.eq(1),
                            ]
                            m.next = "READ_INSTRUCTION"
                    with m.Else():
                        m.d.sync += self.synchronized.eq(0)
                        m.next = "WAIT_END"
                with m.Else():
                    m.d.sync += tickcounter.eq(tickcounter + 1)
            with m.State("READ_INSTRUCTION"):
                m.d.sync += [
                    self.read_en.eq(0),
                    tickcounter.eq(tickcounter + 1),
                ]
                with m.If(read_data[0:8] == INSTRUCTIONS.SCANLINE):
                    with m.If(scanlinenumber < 255):
                        m.d.sync += scanlinenumber.eq(scanlinenumber + 1)
                    with m.Else():
                        m.d.sync += scanlinenumber.eq(0)
                    m.d.sync += [
                        write_data_2.eq(scanlinenumber),
                        self.dir.eq(read_data[8]),
                        stephalfperiod.eq(read_data[9:]),
                    ]
                    m.next = "WAIT_FOR_DATA_RUN"
                with m.Elif(read_data == INSTRUCTIONS.LASTSCANLINE):
                    m.d.sync += [
                        self.expose_finished.eq(1),
                        self.read_commit.eq(1),
                        self.process_lines.eq(0),
                    ]
                    m.next = "WAIT_END"
                with m.Else():
                    m.d.sync += self.error.eq(1)
                    m.next = "READ_INSTRUCTION"
            with m.State("WAIT_FOR_DATA_RUN"):
                m.d.sync += [
                    tickcounter.eq(tickcounter + 1),
                    readbit.eq(0),
                    scanbit.eq(0),
                    lasercnt.eq(0),
                ]
                tickcnt_thresh = int(dct["START%"] * dct["TICKSINFACET"])
                assert tickcnt_thresh > 0
                with m.If(tickcounter >= tickcnt_thresh):
                    m.d.sync += [self.read_en.eq(1), self.write_en_2.eq(1)]
                    m.next = "DATA_RUN"
            with m.State("DATA_RUN"):
                m.d.sync += [tickcounter.eq(tickcounter + 1),
                             self.lasers[1].eq(self.lasers[0])]
                # NOTE:
                #      readbit is your current position in memory
                #      scanbit current byte position in scanline
                #      lasercnt used to pulse laser at certain freq
                with m.If(lasercnt == 0):
                    with m.If(stepcnt >= stephalfperiod):
                        m.d.sync += [self.step.eq(~self.step), stepcnt.eq(0)]
                    with m.Else():
                        m.d.sync += stepcnt.eq(stepcnt + 1)
                    with m.If(scanbit >= dct["BITSINSCANLINE"]):
                        m.d.sync += [
                            self.write_commit_2.eq(1),
                            self.lasers[0].eq(0),
                        ]
                        with m.If(dct["SINGLE_LINE"] & self.empty):
                            m.d.sync += self.read_discard.eq(1)
                        with m.Else():
                            m.d.sync += self.read_commit.eq(1)
                        m.next = "WAIT_END"
                    with m.Else():
                        m.d.sync += [
                            lasercnt.eq(dct["LASERTICKS"] - 1),
                            scanbit.eq(scanbit + 1),
                        ]
                        m.d.sync += write_new[0].eq(self.photodiode_2)
                        with m.If(readbit == 0):
                            m.d.sync += [
                                self.lasers[0].eq(read_data[0]),
                                read_old.eq(read_data >> 1),
                                self.read_en.eq(0),
                                self.write_en_2.eq(0),
                            ]
                        with m.Elif(readbit == MEMWIDTH - 1):
                            m.d.sync += [
                                write_data_2.eq(write_new),
                                self.lasers[0].eq(read_old[0]),
                            ]
                        with m.Else():
                            m.d.sync += self.lasers[0].eq(read_old[0])
                with m.Else():
                    m.d.sync += lasercnt.eq(lasercnt - 1)
                    # NOTE: read enable can only be high for 1 cycle
                    #       as a result this is done right before the "read"
                    with m.If(lasercnt == 1):
                        with m.If(readbit == 0):
                            m.d.sync += [readbit.eq(readbit + 1)]
                        # final read bit copy memory
                        # move to next address, i.e. byte, if end is reached
                        with m.Elif(readbit == MEMWIDTH - 1):
                            # If fifo is empty it will give errors later
                            # so it can be ignored here
                            # Only grab a new line if more than current
                            # is needed
                            # -1 as counting in python is different
                            with m.If(scanbit < (dct["BITSINSCANLINE"])):
                                m.d.sync += [
                                    self.read_en.eq(1),
                                    self.write_en_2.eq(1),
                                ]
                            m.d.sync += readbit.eq(0)
                        with m.Else():
                            m.d.sync += [
                                readbit.eq(readbit + 1),
                                read_old.eq(read_old >> 1),
                            ]
            with m.State("WAIT_END"):
                m.d.sync += [
                    tickcounter.eq(tickcounter + 1),
                    self.write_commit_2.eq(0),
                ]
                with m.If(dct["SINGLE_LINE"] & self.empty):
                    m.d.sync += self.read_discard.eq(0)
                with m.Else():
                    m.d.sync += self.read_commit.eq(0)
                
                # following lines can be used to test exposure width during
                # LaserHeadTest.test_stable
                # start_line = int(dct["START%"] * dct["TICKSINFACET"])
                # end_line = int(dct["END%"] * dct["TICKSINFACET"]) 
                # with m.If((tickcounter >= start_line) & (tickcounter <= end_line) & (self.synchronized == 1)):
                #     m.d.sync += lasers.eq(int("11", 2))
                # -1 as you count till range-1 in python
                # -2 as you need 1 tick to process
                with m.If(
                    tickcounter
                    >= round(dct["TICKSINFACET"] - dct["JITTERTICKS"] - 2)
                ):
                    m.d.sync += lasers.eq(int("10", 2))
                    m.next = "WAIT_STABLE"
                with m.Else():
                    m.d.sync += lasers.eq(int("00", 2))
                # if user disables synhcronization exit
                with m.If(~self.synchronize):
                    m.next = "STOP"
        if self.platform.name == "Test":
            self.laserfsm = laserfsm
        return m

class DiodeSimulator(Laserhead):
    """Wraps laser head with object to simulate photodiode

    This is purely used for testing. Photodiode is only created
    if prism motor is enabled and the laser is on so the diode
    can be triggered.
    """

    def __init__(self, platform, laser_var=None, addfifo=True):
        if laser_var is not None:
            platform.laser_var = laser_var
        super().__init__(platform)
        self.addfifo = addfifo
        self.laser0in = Signal()
        self.laser1in = Signal()
        self.enable_prism_in = Signal()
        if addfifo:
            self.write_en = Signal()
            self.write_commit = Signal()
            self.write_data = Signal(MEMWIDTH)
            self.read_en_2 = Signal()
            self.read_commit_2 = Signal()
            self.read_data_2 = Signal(MEMWIDTH)

    def elaborate(self, platform):
        if self.platform is not None:
            platform = self.platform
        m = super().elaborate(platform)

        dct = self.dct
        diodecounter = Signal(range(dct["TICKSINFACET"]))
        self.diodecounter = diodecounter

        if self.addfifo:
            m.d.comb += [
                self.enable_prism_in.eq(self.enable_prism),
                self.laser0in.eq(self.lasers[0]),
                self.laser1in.eq(self.lasers[1]),
            ]
            # fifo 1:
            fifo = TransactionalizedFIFO(
                width=MEMWIDTH, depth=platform.memdepth
            )
            m.submodules.fifo = fifo
            self.fifo = fifo
            m.d.comb += [
                fifo.write_data.eq(self.write_data),
                fifo.write_commit.eq(self.write_commit),
                fifo.write_en.eq(self.write_en),
                fifo.read_commit.eq(self.read_commit),
                fifo.read_en.eq(self.read_en),
                self.empty.eq(fifo.empty),
                fifo.read_discard.eq(self.read_discard),
                self.read_data.eq(fifo.read_data),
            ]
            # fifo 2:
            fifo2 = TransactionalizedFIFO(
                width=MEMWIDTH, depth=platform.memdepth
            )
            m.submodules.fifo2 = fifo2
            self.fifo2 = fifo2
            m.d.comb += [
                fifo2.write_data.eq(self.write_data_2),
                fifo2.write_commit.eq(self.write_commit_2),
                fifo2.write_en.eq(self.write_en_2),
                fifo2.read_commit.eq(self.read_commit_2),
                fifo2.read_en.eq(self.read_en_2),
                self.full.eq(fifo2.full),
                fifo2.write_discard.eq(self.write_discard_2),
                self.read_data_2.eq(fifo2.read_data),
            ]

        with m.If(diodecounter == (dct["TICKSINFACET"] - 1)):
            m.d.sync += diodecounter.eq(0)
        with m.Elif(diodecounter > (dct["TICKSINFACET"] - 4)):
            m.d.sync += [
                self.photodiode.eq(
                    ~(self.enable_prism_in & (self.laser0in | self.laser1in))
                ),
                diodecounter.eq(diodecounter + 1),
            ]
        with m.Else():
            m.d.sync += [
                diodecounter.eq(diodecounter + 1),
                self.photodiode.eq(1),
            ]
        self.diodecounter = diodecounter
        return m

