import spidev

spi = spidev.SpiDev()
spi.open(0, 0)
spi.no_cs = True
spi.mode = 0
spi.max_speed_hz = round(1E6)
for _ in range(3):
    spi.cshigh = False
    print(spi.xfer([10]))
    print(spi.xfer([10]))
    spi.cshigh = True
    print(spi.xfer([10]))
    print(spi.xfer([10]))
    spi.cshigh = False
    print(spi.xfer([10]))
    print(spi.xfer([10]))
    spi.cshigh = True
    print(spi.xfer([10]))
    print(spi.xfer([10]))

# at the start of transaction
#  spi.cs is high !


# at the end of tranasction cs high is low