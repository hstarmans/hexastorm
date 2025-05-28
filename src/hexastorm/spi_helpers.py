from amaranth import Signal
from amaranth.lib.cdc import FFSynchronizer


def connect_synchronized_spi(m, board_spi, spi_interface):
    """Connect and synchronize SPI pins to a Luna SPI interface.

    Parameters:
        m              -- Amaranth module
        board_spi      -- Requested resource (with .sck/.cs/.sdi/.sdo)
        spi_interface  -- An instance of Luna's SPIDeviceInterface
    """

    # Create synchronized input signals
    synced_clk  = Signal()
    synced_csn  = Signal()
    synced_mosi = Signal()

    # Add synchronizers
    m.submodules += [
        FFSynchronizer(board_spi.sck.i,  synced_clk),
        FFSynchronizer(board_spi.cs.i,   synced_csn),
        FFSynchronizer(board_spi.sdi.i,  synced_mosi)
    ]

    # Wire up to the SPI interface
    m.d.comb += [
        spi_interface.spi.sck.eq(synced_clk),
        spi_interface.spi.cs.eq(synced_csn),
        spi_interface.spi.sdi.eq(synced_mosi),
        board_spi.sdo.o.eq(spi_interface.spi.sdo)
    ]
