from amaranth import Signal
from amaranth.lib.cdc import FFSynchronizer
from amaranth.lib.io import Buffer


def connect_synchronized_spi(m, board_spi, spi_interface, singlestage_sync=False):
    """Connect and synchronize SPI pins to a Luna SPI interface.

    Parameters:
        m              -- Amaranth module
        board_spi      -- Requested resource (with .sck/.cs/.sdi/.sdo), requested with dir="-"
        spi_interface  -- An instance of Luna's SPIDeviceInterface
        singestage_sync -- If True, use single-stage synchronization (may be less reliable)
    """

    # Wrap I/O pins with direction-aware buffers
    sck = Buffer("i", board_spi.sck)
    cs = Buffer("i", board_spi.cs)
    sdi = Buffer("i", board_spi.sdi)
    sdo = Buffer("o", board_spi.sdo)

    # Register buffers as submodules
    m.submodules += [sck, cs, sdi, sdo]

    if not singlestage_sync:
        # Create synchronized input signals
        synced_clk = Signal()
        synced_csn = Signal()
        synced_mosi = Signal()

        # Add synchronizers
        m.submodules += [
            FFSynchronizer(sck.i, synced_clk),
            FFSynchronizer(cs.i, synced_csn),
            FFSynchronizer(sdi.i, synced_mosi),
        ]

        # Wire up to the SPI interface
        m.d.comb += [
            spi_interface.spi.sck.eq(synced_clk),
            spi_interface.spi.cs.eq(synced_csn),
            spi_interface.spi.sdi.eq(synced_mosi),
            sdo.o.eq(spi_interface.spi.sdo),
        ]
    else:
        # Single stage synchronization works in practice
        # Create single-stage registered signals
        synced_sck = Signal()
        synced_cs = Signal()
        synced_sdi = Signal()

        # Register inputs on the FPGA clock
        m.d.sync += [
            synced_sck.eq(sck.i),
            synced_cs.eq(cs.i),
            synced_sdi.eq(sdi.i),
        ]

        # Connect the registered signals to the SPI interface
        m.d.comb += [
            spi_interface.spi.sck.eq(synced_sck),
            spi_interface.spi.cs.eq(synced_cs),
            spi_interface.spi.sdi.eq(synced_sdi),
            sdo.o.eq(spi_interface.spi.sdo),
        ]
