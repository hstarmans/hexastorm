from amaranth import Elaboratable, Module, Signal, Cat
from luna.gateware.memory import TransactionalizedFIFO

from .config import INSTRUCTIONS, MEMWIDTH, params


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
    O: read_commit    -- finalize read transactionalizedfifo
    O: read_en        -- enable read transactionalizedfifo
    I: read_data      -- read data from transactionalizedfifo, AKA busy
    I: empty          -- signal wether fifo is empty
    O: step           -- step signal
    O: direction      -- direction signal
    """

    def __init__(self, platform):
        self.platform = platform
        self.dct = params(platform)

        # Control and status signals
        self.synchronize = Signal()
        self.synchronized = Signal()
        self.singlefacet = Signal()
        self.expose_start = Signal()
        self.expose_finished = Signal()
        self.error = Signal()
        self.process_lines = Signal()
        self.ticksinfacet = Signal(range(self.dct["TICKSINFACET"] * 2))

        # Motor and laser control
        self.lasers = Signal(2)
        self.pwm = Signal()
        self.enable_prism = Signal()
        self.step = Signal()
        self.dir = Signal()

        # Photodiode signals
        self.photodiode = Signal()
        self.photodiode_t = Signal()

        # FIFO memory interface (read)
        self.read_commit = Signal()
        self.read_en = Signal()
        self.read_data = Signal(MEMWIDTH)
        self.read_discard = Signal()
        self.empty = Signal()

        # FIFO memory interface (write)
        self.write_commit_2 = Signal()
        self.write_en_2 = Signal()
        self.write_data_2 = Signal(MEMWIDTH)
        self.write_discard_2 = Signal()
        self.full = Signal()

    def elaborate(self, platform):
        m = Module()
        dct = self.dct

        # Pulse generator for prism motor
        pwm_counter = Signal(range(dct["POLYPERIOD"]))
        with m.If(pwm_counter == dct["POLYPERIOD"] - 1):
            m.d.sync += [
                self.pwm.eq(~self.pwm),
                pwm_counter.eq(0),
            ]
        with m.Else():
            m.d.sync += pwm_counter.eq(pwm_counter + 1)

        # Photodiode edge detector:
        # Sets `photodiode_t` high if the photodiode
        # was low at any point during a tick window.
        # Used for photodiode test validation.
        phtd_cnt_max = dct["TICKSINFACET"] * 2
        phtd_cnt = Signal(range(phtd_cnt_max))
        phtd_triggered = Signal()

        with m.If(phtd_cnt < (phtd_cnt_max - 1)):
            m.d.sync += [
                phtd_cnt.eq(phtd_cnt + 1),
                phtd_triggered.eq(phtd_triggered | ~self.photodiode),
            ]
        with m.Else():
            m.d.sync += [
                self.photodiode_t.eq(phtd_triggered),
                phtd_cnt.eq(0),
                phtd_triggered.eq(0),
            ]

        # Detect rising edge on expose_start
        # On rising edge of expose_start: start printing and assert process_lines
        expose_start_prev = Signal()
        m.d.sync += expose_start_prev.eq(self.expose_start)

        with m.If(~expose_start_prev & self.expose_start):  # Rising edge
            m.d.sync += [
                self.process_lines.eq(1),
                self.expose_finished.eq(0),
            ]

        # step generator, i.e. slowest speed is 1/(2^4-1)
        stephalfperiod = Signal(dct["BITSINSCANLINE"].bit_length() + 4)
        stepcnt = Signal.like(stephalfperiod)

        # Laser FSM
        # syncfailed is lowered once synchronized
        syncfailed_cnt = Signal(range(dct["STABLETICKS"]))
        facetcnt = Signal(range(dct["FACETS"]))
        lasercnt = Signal(range(dct["LASERTICKS"]))
        byte_index = Signal(range(dct["BITSINSCANLINE"] + 1))
        tickcounter = Signal(range(max(dct["SPINUPTICKS"], dct["STABLETICKS"])))

        photodiode = self.photodiode
        read_data = self.read_data
        read_old = Signal.like(read_data)
        bit_index = Signal(range(MEMWIDTH))
        photodiode_d = Signal()
        lasers = self.lasers

        with m.FSM(init="RESET") as laserfsm:
            with m.State("RESET"):
                m.d.sync += [
                    self.error.eq(0),
                    self.synchronized.eq(0),
                    self.enable_prism.eq(0),
                    self.lasers.eq(0),
                ]
                m.next = "STOP"

            with m.State("STOP"):
                m.d.sync += [
                    syncfailed_cnt.eq(dct["STABLETICKS"] - 1),
                    tickcounter.eq(0),
                    facetcnt.eq(0),
                    self.synchronized.eq(0),
                    self.enable_prism.eq(0),
                    bit_index.eq(0),
                    byte_index.eq(0),
                    lasercnt.eq(0),
                    lasers.eq(0),
                ]
                with m.If(self.synchronize & (~self.error)):
                    # Error: photodiode cannot be high without active laser
                    with m.If(self.photodiode == 0):
                        m.d.sync += self.error.eq(1)
                    with m.Else():
                        m.d.sync += [self.error.eq(0), self.enable_prism.eq(1)]
                        m.next = "SPINUP"

            with m.State("SPINUP"):
                with m.If(tickcounter > dct["SPINUPTICKS"] - 1):
                    # turn on single channel
                    m.d.sync += [
                        self.lasers.eq(0b10),
                        tickcounter.eq(0),
                    ]
                    m.next = "WAIT_STABLE"
                with m.Else():
                    m.d.sync += tickcounter.eq(tickcounter + 1)

            with m.State("WAIT_STABLE"):
                # Store previous photodiode value
                m.d.sync += [photodiode_d.eq(photodiode), self.write_en_2.eq(1)]

                # Timeout: photodiode didn't fall in time
                with m.If(tickcounter >= syncfailed_cnt):
                    m.d.sync += [self.error.eq(1), self.write_en_2.eq(0)]
                    m.next = "STOP"

                # Falling edge detected (photodiode: 1 → 0)
                with m.Elif(~photodiode & ~photodiode_d):
                    m.d.sync += [
                        tickcounter.eq(0),
                        lasers.eq(0),
                    ]

                    # Check if synchronization timing is within expected range
                    sync_margin = (dct["TICKSINFACET"] - 1) - dct["JITTERTICKS"]
                    with m.If(tickcounter > sync_margin):
                        m.d.sync += [
                            self.synchronized.eq(1),
                            self.ticksinfacet.eq(tickcounter),
                            self.write_data_2.eq(Cat(facetcnt, tickcounter)),
                            self.write_en_2.eq(0),
                        ]

                        # Increment or reset facet counter
                        with m.If(facetcnt == dct["FACETS"] - 1):
                            m.d.sync += facetcnt.eq(0)
                        with m.Else():
                            m.d.sync += facetcnt.eq(facetcnt + 1)

                        # Exit early if only a single facet should be scanned
                        with m.If(self.singlefacet & (facetcnt > 0)):
                            m.next = "WAIT_END"
                        # Exit if FIFO is empty or scanning is complete
                        with m.Elif(self.empty | ~self.process_lines):
                            m.next = "WAIT_END"
                        # Proceed to read instruction
                        with m.Else():
                            # TODO: 10 is too high, should be lower
                            syncfailed_sync_max = min(
                                round(10.1 * dct["TICKSINFACET"]),
                                dct["STABLETICKS"],
                            )
                            m.d.sync += [
                                syncfailed_cnt.eq(syncfailed_sync_max),
                                self.read_en.eq(1),
                            ]
                            m.next = "READ_INSTRUCTION"
                    # Not synchronized — too early
                    with m.Else():
                        m.d.sync += [self.synchronized.eq(0), self.write_en_2.eq(0)]
                        m.next = "WAIT_END"
                # No event yet, just increment tick counter
                with m.Else():
                    m.d.sync += tickcounter.eq(tickcounter + 1)

            with m.State("READ_INSTRUCTION"):
                m.d.sync += [
                    self.read_en.eq(0),
                    tickcounter.eq(tickcounter + 1),
                ]
                instruction = read_data[:8]
                with m.Switch(instruction):
                    with m.Case(INSTRUCTIONS.SCANLINE):
                        m.d.sync += [
                            self.dir.eq(read_data[8]),
                            stephalfperiod.eq(read_data[9:]),
                        ]
                        m.next = "WAIT_FOR_DATA_RUN"
                    with m.Case(INSTRUCTIONS.LASTSCANLINE):
                        m.d.sync += [
                            self.expose_finished.eq(1),
                            self.read_commit.eq(1),
                            self.process_lines.eq(0),
                        ]
                        m.next = "WAIT_END"
                    with m.Default():
                        m.d.sync += self.error.eq(1)
                        m.next = "READ_INSTRUCTION"

            with m.State("WAIT_FOR_DATA_RUN"):
                m.d.sync += [
                    tickcounter.eq(tickcounter + 1),
                    bit_index.eq(0),
                    byte_index.eq(0),
                    lasercnt.eq(0),
                ]
                start_threshold = int(dct["START%"] * dct["TICKSINFACET"])
                assert start_threshold > 0
                with m.If(tickcounter >= start_threshold):
                    m.d.sync += self.read_en.eq(1)
                    m.next = "DATA_RUN"

            with m.State("DATA_RUN"):
                m.d.sync += [
                    tickcounter.eq(tickcounter + 1),
                    self.lasers[1].eq(self.lasers[0]),
                ]
                # NOTE:
                #      lasercnt used to pulse laser at certain freq
                with m.If(lasercnt == 0):
                    # Handle motor stepping
                    with m.If(stepcnt >= stephalfperiod):
                        m.d.sync += [self.step.eq(~self.step), stepcnt.eq(0)]
                    with m.Else():
                        m.d.sync += stepcnt.eq(stepcnt + 1)

                    # End of scanline reached
                    with m.If(byte_index >= dct["BITSINSCANLINE"]):
                        m.d.sync += (self.lasers[0].eq(0),)

                        # Commit or discard based on configuration and FIFO status
                        with m.If(dct["SINGLE_LINE"] & self.empty):
                            m.d.sync += self.read_discard.eq(1)
                        with m.Else():
                            m.d.sync += self.read_commit.eq(1)

                        m.next = "WAIT_END"

                    # Still scanning — advance and output laser bit
                    with m.Else():
                        m.d.sync += [
                            lasercnt.eq(dct["LASERTICKS"] - 1),
                            byte_index.eq(byte_index + 1),
                        ]
                        with m.If(bit_index == 0):
                            m.d.sync += [
                                self.lasers[0].eq(read_data[0]),
                                read_old.eq(read_data >> 1),
                                self.read_en.eq(0),
                            ]
                        with m.Else():
                            m.d.sync += self.lasers[0].eq(read_old[0])
                with m.Else():
                    m.d.sync += lasercnt.eq(lasercnt - 1)
                    # NOTE: read enable can only be high for 1 cycle
                    #       as a result this is done right before the "read"
                    with m.If(lasercnt == 1):
                        # Advance read bit position
                        with m.If(bit_index == 0):
                            m.d.sync += [bit_index.eq(bit_index + 1)]
                        # Last bit in word — fetch next byte if needed
                        with m.Elif(bit_index == MEMWIDTH - 1):
                            # If fifo is empty it will give errors later
                            # so it can be ignored here
                            # Only grab a new line if more than current
                            # is needed
                            # -1 as counting in python is different
                            with m.If(byte_index < (dct["BITSINSCANLINE"])):
                                m.d.sync += self.read_en.eq(1)
                            m.d.sync += bit_index.eq(0)
                        with m.Else():
                            m.d.sync += [
                                bit_index.eq(bit_index + 1),
                                read_old.eq(read_old >> 1),
                            ]
            with m.State("WAIT_END"):
                m.d.sync += [
                    tickcounter.eq(tickcounter + 1),
                    self.write_commit_2.eq(1),
                ]
                # Decide whether to commit or discard the current line
                with m.If(dct["SINGLE_LINE"] & self.empty):
                    m.d.sync += self.read_discard.eq(0)
                with m.Else():
                    m.d.sync += self.read_commit.eq(0)

                # -1 as you count till range-1 in python
                # -2 as you need 1 tick to process
                exposure_end = round(dct["TICKSINFACET"] - dct["JITTERTICKS"] - 2)
                with m.If(tickcounter >= exposure_end):
                    m.d.sync += [lasers.eq(0b10), self.write_commit_2.eq(0)]
                    m.next = "WAIT_STABLE"
                with m.Else():
                    m.d.sync += lasers.eq(0b00)

                # If user disables synchronization, stop immediately
                with m.If(~self.synchronize):
                    m.d.sync += self.write_commit_2.eq(0)
                    m.next = "STOP"

        if self.platform.name == "Test":
            self.stephalfperiod = stephalfperiod
            self.tickcounter = tickcounter
            self.scanbit = byte_index
            self.lasercnt = lasercnt
            self.facetcnt = facetcnt
            self.laserfsm = laserfsm
        return m


class DiodeSimulator(Laserhead):
    """Laserhead wrapper for simulating a photodiode in test environments.

    The simulated photodiode goes low (active) only when the prism motor
    is enabled and the laser is on so the diode
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
            # FIFO 1:
            fifo = TransactionalizedFIFO(width=MEMWIDTH, depth=platform.memdepth)
            m.submodules.fifo = fifo
            self.fifo = fifo

            m.d.comb += [
                fifo.write_data.eq(self.write_data),
                fifo.write_commit.eq(self.write_commit),
                fifo.write_en.eq(self.write_en),
                fifo.read_commit.eq(self.read_commit),
                fifo.read_en.eq(self.read_en),
                fifo.read_discard.eq(self.read_discard),
                self.empty.eq(fifo.empty),
                self.read_data.eq(fifo.read_data),
            ]
            # FIFO 2:
            fifo2 = TransactionalizedFIFO(width=MEMWIDTH, depth=platform.memdepth)
            m.submodules.fifo2 = fifo2
            self.fifo2 = fifo2

            m.d.comb += [
                fifo2.write_data.eq(self.write_data_2),
                fifo2.write_commit.eq(self.write_commit_2),
                fifo2.write_discard.eq(self.write_discard_2),
                fifo2.write_en.eq(self.write_en_2),
                fifo2.read_commit.eq(self.read_commit_2),
                fifo2.read_en.eq(self.read_en_2),
                self.full.eq(fifo2.full),
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
        return m
