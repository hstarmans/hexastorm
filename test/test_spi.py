import spidev
from time import sleep

spi = spidev.SpiDev()
spi.open(0, 0)
spi.no_cs = False
spi.mode = 1
spi.max_speed_hz = round(1E6)
bts = [210, 222, 230]
for idx, byte in enumerate(bts):
    spi.cshigh = True
    if idx != 0:
        assert current == spi.xfer([byte])[0]
    else:
        spi.xfer([byte])
    spi.cshigh = False
    current = byte
# at the start of transaction
#  spi.cs is high !


# at the end of tranasction cs high is low