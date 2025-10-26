from amaranth import *
from amaranth.lib.cdc import PulseSynchronizer


class SPIBus(Record):
    def __init__(self):
        super().__init__([("sck", 1), ("cs", 1), ("sdi", 1), ("sdo", 1)])


class SPIDeviceInterfaceFast(Elaboratable):
    """Higher-speed SPI slave that synchronizes only on word-complete."""

    def __init__(
        self,
        *,
        word_size=8,
        clock_polarity=0,
        clock_phase=0,
        msb_first=True,
        cs_idles_high=False,
    ):
        self.word_size = word_size
        self.clock_polarity = clock_polarity
        self.clock_phase = clock_phase
        self.msb_first = msb_first
        self.cs_idles_high = cs_idles_high

        # SPI I/O
        self.spi = SPIBus()

        # Data interface
        self.word_in = Signal(word_size)
        self.word_out = Signal(word_size)
        self.word_complete = Signal()
        self.word_accepted = Signal()

    def elaborate(self, platform):
        m = Module()

        # Edge detector for SCK (works up to ~sysclk/2)
        serial_clock = Signal()
        if self.clock_polarity:
            m.d.comb += serial_clock.eq(~self.spi.sck)
        else:
            m.d.comb += serial_clock.eq(self.spi.sck)

        prev_clk = Signal()
        m.d.sync += prev_clk.eq(serial_clock)

        leading_edge = ~prev_clk & serial_clock
        trailing_edge = prev_clk & ~serial_clock

        sample_edge = trailing_edge if self.clock_phase else leading_edge
        output_edge = leading_edge if self.clock_phase else trailing_edge

        chip_selected = ~self.spi.cs if not self.cs_idles_high else self.spi.cs

        # Shift registers
        bit_count = Signal(range(self.word_size + 1))
        rx_shift = Signal(self.word_size)
        tx_shift = Signal(self.word_size)

        word_ready_spi = Signal()

        with m.If(chip_selected):
            # Shift in
            with m.If(sample_edge):
                if self.msb_first:
                    m.d.sync += rx_shift.eq(Cat(self.spi.sdi, rx_shift[:-1]))
                else:
                    m.d.sync += rx_shift.eq(Cat(rx_shift[1:], self.spi.sdi))

                with m.If(bit_count == self.word_size - 1):
                    m.d.sync += [
                        bit_count.eq(0),
                        word_ready_spi.eq(1),
                        tx_shift.eq(self.word_out),
                    ]
                with m.Else():
                    m.d.sync += [bit_count.eq(bit_count + 1), word_ready_spi.eq(0)]

            # Shift out
            with m.If(output_edge):
                if self.msb_first:
                    m.d.sync += Cat(tx_shift[1:], self.spi.sdo).eq(tx_shift)
                else:
                    m.d.sync += Cat(self.spi.sdo, tx_shift[:-1]).eq(tx_shift)

        with m.Else():
            m.d.sync += [
                tx_shift.eq(self.word_out),
                bit_count.eq(0),
                word_ready_spi.eq(0),
            ]

        # Pulse synchronizer ensures one clean pulse per word
        ps = m.submodules.ps = PulseSynchronizer("sync", "sync")
        m.d.comb += ps.i.eq(word_ready_spi)
        synced_ready = ps.o

        with m.If(synced_ready):
            m.d.sync += [
                self.word_in.eq(rx_shift),
                self.word_complete.eq(1),
                self.word_accepted.eq(1),
            ]
        with m.Else():
            m.d.sync += [self.word_complete.eq(0), self.word_accepted.eq(0)]

        return m
