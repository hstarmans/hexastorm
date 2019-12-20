# Run on raspberry pi
import spidev
spi = spidev.SpiDev()
# first zero is device, second zero is chip select (raspberry pi has 2 chip selects)
spi.open(0,0)
spi.max_speed_hz = 1000000
spi.xfer([0x03])
spi.xfer([0x03])
assert spi.xfer([0x01]) == [0]
assert spi.xfer([0x01]) == [2]  
assert spi.xfer([0x02]) == [2]
assert spi.xfer([0x02]) == [8] 