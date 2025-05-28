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

    # in luna record objects signals have direction
    # this is no longer needed and fixed below

    def _get_input_signal(signal):
        return signal.i if hasattr(signal, "i") else signal

    def _get_output_signal(signal):
        return signal.o if hasattr(signal, "o") else signal
    


    # Add synchronizers
    m.submodules += [
        FFSynchronizer(_get_input_signal(board_spi.sck),  synced_clk),
        FFSynchronizer(_get_input_signal(board_spi.cs),   synced_csn),
        FFSynchronizer(_get_input_signal(board_spi.sdi),  synced_mosi)
    ]

    # Wire up to the SPI interface
    m.d.comb += [
        spi_interface.spi.sck.eq(synced_clk),
        spi_interface.spi.cs.eq(synced_csn),
        spi_interface.spi.sdi.eq(synced_mosi),
        _get_output_signal(board_spi.sdo).eq(spi_interface.spi.sdo)
    ]#
