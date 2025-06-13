from amaranth import Elaboratable, Module, Signal, Cat
from luna.gateware.memory import TransactionalizedFIFO

from .config import Spi


class Laserhead(Elaboratable):
    """
    Controller for laser scanning systems using a rotating mirror or prism.

    This module manages synchronization with a photodiode, precise timing of laser
    exposure, scanline processing from FIFO, and motor stepping logic. It supports
    single-facet and multi-facet scanning modes.

    Inputs:
        synchronize     -- Start/enable synchronization process.
        singlefacet     -- Limit operation to a single facet.
        expose_start    -- Start exposing scanlines.
        photodiode      -- Photodiode input for synchronization.
        read_data       -- Data from scanline FIFO.
        empty           -- FIFO empty flag.

    Outputs:
        synchronized    -- High when synchronized with photodiode signal.
        expose_finished -- High when all scanlines have been exposed.
        error           -- Indicates synchronization or data error.
        lasers          -- Laser output signal (2-bit).
        pwm             -- Motor PWM output.
        enable_prism    -- Enable signal for motor driver.
        photodiode_t    -- High if photodiode triggered this cycle.
        read_en         -- Read enable signal for FIFO.
        read_commit     -- Commit current line in FIFO.
        step            -- Step signal for motor.
        dir             -- Stepper motor direction.
    """

    def __init__(self, platform):
        self.platform = platform
        hdl_cfg = platform.hdl_cfg
        laz_tim = platform.settings.laser_timing

        # Control and status signals
        self.synchronize = Signal()
        self.synchronized = Signal()
        self.singlefacet = Signal()
        self.expose_start = Signal()
        self.expose_finished = Signal()
        self.error = Signal()
        self.process_lines = Signal()
        self.facet_ticks = Signal(range(laz_tim["facet_ticks"] * 2))

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
        self.read_data = Signal(hdl_cfg.mem_width)
        self.read_discard = Signal()
        self.empty = Signal()

        # FIFO memory interface (write)
        self.write_commit_2 = Signal()
        self.write_en_2 = Signal()
        self.write_data_2 = Signal(hdl_cfg.mem_width)
        self.write_discard_2 = Signal()
        self.full = Signal()

    def elaborate(self, platform):
        m = Module()
        plf = self.platform or platform
        laz_tim = plf.settings.laser_timing
        hdl_cfg = plf.hdl_cfg

        # Pulse generator for prism motor
        pwm_counter = Signal(range(laz_tim["motor_period"]))
        with m.If(pwm_counter == laz_tim["motor_period"] - 1):
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
        phtd_cnt_max = laz_tim["facet_ticks"] * 2
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
        stephalfperiod = Signal(laz_tim["scanline_length"].bit_length() + 4)
        stepcnt = Signal.like(stephalfperiod)

        # Laser FSM
        # syncfailed is lowered once synchronized
        syncfailed_cnt = Signal(range(laz_tim["stable_ticks"]))
        facetcnt = Signal(range(laz_tim["facets"]))
        lasercnt = Signal(range(laz_tim["laser_ticks"]))
        byte_index = Signal(range(laz_tim["scanline_length"] + 1))
        tickcounter = Signal(
            range(max(laz_tim["spinup_ticks"], laz_tim["stable_ticks"]))
        )

        photodiode = self.photodiode
        read_data = self.read_data
        read_old = Signal.like(read_data)
        bit_index = Signal(range(hdl_cfg.mem_width))
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
                    syncfailed_cnt.eq(laz_tim["stable_ticks"] - 1),
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
                with m.If(tickcounter > laz_tim["spinup_ticks"] - 1):
                    # turn on single channel
                    m.d.sync += [
                        lasers.eq(0b10),
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
                    sync_margin = (laz_tim["facet_ticks"] - 1) - laz_tim["jitter_ticks"]
                    with m.If(tickcounter > sync_margin):
                        m.d.sync += [
                            self.synchronized.eq(1),
                            self.facet_ticks.eq(tickcounter),
                            self.write_data_2.eq(Cat(facetcnt, tickcounter)),
                            self.write_en_2.eq(0),
                        ]

                        # Increment or reset facet counter
                        with m.If(facetcnt == laz_tim["facets"] - 1):
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
                                round(10.1 * laz_tim["facet_ticks"]),
                                laz_tim["stable_ticks"],
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
                    with m.Case(Spi.Instructions.scanline):
                        m.d.sync += [
                            self.dir.eq(read_data[8]),
                            stephalfperiod.eq(read_data[9:]),
                        ]
                        m.next = "WAIT_FOR_DATA_RUN"
                    with m.Case(Spi.Instructions.last_scanline):
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
                start_threshold = int(laz_tim["start_frac"] * laz_tim["facet_ticks"])
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
                    with m.If(byte_index >= laz_tim["scanline_length"]):
                        m.d.sync += (self.lasers[0].eq(0),)

                        # Commit or discard based on configuration and FIFO status
                        with m.If(hdl_cfg.single_line & self.empty):
                            m.d.sync += self.read_discard.eq(1)
                        with m.Else():
                            m.d.sync += self.read_commit.eq(1)

                        m.next = "WAIT_END"

                    # Still scanning — advance and output laser bit
                    with m.Else():
                        m.d.sync += [
                            lasercnt.eq(laz_tim["laser_ticks"] - 1),
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
                        with m.Elif(bit_index == hdl_cfg.mem_width - 1):
                            # If fifo is empty it will give errors later
                            # so it can be ignored here
                            # Only grab a new line if more than current
                            # is needed
                            # -1 as counting in python is different
                            with m.If(byte_index < (laz_tim["scanline_length"])):
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
                with m.If(hdl_cfg.single_line & self.empty):
                    m.d.sync += self.read_discard.eq(0)
                with m.Else():
                    m.d.sync += self.read_commit.eq(0)

                # -1 as you count till range-1 in python
                # -2 as you need 1 tick to process
                exposure_end = round(
                    laz_tim["facet_ticks"] - laz_tim["jitter_ticks"] - 2
                )
                with m.If(tickcounter >= exposure_end):
                    m.d.sync += [lasers.eq(0b10), self.write_commit_2.eq(0)]
                    m.next = "WAIT_STABLE"
                with m.Else():
                    m.d.sync += lasers.eq(0b00)

                # If user disables synchronization, stop immediately
                with m.If(~self.synchronize):
                    m.d.sync += self.write_commit_2.eq(0)
                    m.next = "STOP"

        if self.platform.settings.test:
            self.stephalfperiod = stephalfperiod
            self.tickcounter = tickcounter
            self.scanbit = byte_index
            self.lasercnt = lasercnt
            self.facetcnt = facetcnt
            self.laserfsm = laserfsm
        return m


class DiodeSimulator(Laserhead):
    """
    Simulates a photodiode signal for testing laserhead behavior in a controlled environment.

    The photodiode goes low (active) only when the prism motor is enabled
    and at least one laser channel is on, mimicking real-world diode triggering.
    Optionally includes transactional FIFO buffers for scanline data emulation.
    """

    def __init__(self, platform, addfifo=True):
        hdl_cfg = platform.hdl_cfg
        super().__init__(platform)

        self.addfifo = addfifo
        self.laser0 = Signal()
        self.laser1 = Signal()
        self.enable_prism_in = Signal()

        if addfifo:
            self.write_en = Signal()
            self.write_commit = Signal()
            self.write_data = Signal(hdl_cfg.mem_width)
            self.read_en_2 = Signal()
            self.read_commit_2 = Signal()
            self.read_data_2 = Signal(hdl_cfg.mem_width)

    def elaborate(self, platform):
        platform = self.platform or platform
        m = super().elaborate(platform)
        hdl_cfg = platform.hdl_cfg
        laz_tim = platform.settings.laser_timing

        diode_cnt = Signal(range(laz_tim["facet_ticks"]))
        self.diode_cnt = diode_cnt

        if self.addfifo:
            m.d.comb += [
                self.enable_prism_in.eq(self.enable_prism),
                self.laser0.eq(self.lasers[0]),
                self.laser1.eq(self.lasers[1]),
            ]
            # FIFO 1:
            fifo = TransactionalizedFIFO(
                width=hdl_cfg.mem_width, depth=hdl_cfg.mem_depth
            )
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
            fifo2 = TransactionalizedFIFO(
                width=hdl_cfg.mem_width, depth=hdl_cfg.mem_depth
            )
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

        with m.If(diode_cnt == (laz_tim["facet_ticks"] - 1)):
            m.d.sync += diode_cnt.eq(0)
        with m.Elif(diode_cnt > (laz_tim["facet_ticks"] - laz_tim["facets"])):
            m.d.sync += [
                self.photodiode.eq(
                    ~(self.enable_prism_in & (self.laser0 | self.laser1))
                ),
                diode_cnt.eq(diode_cnt + 1),
            ]
        with m.Else():
            m.d.sync += [
                diode_cnt.eq(diode_cnt + 1),
                self.photodiode.eq(1),
            ]
        return m
