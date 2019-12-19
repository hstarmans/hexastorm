# Run on raspberry pi
import spidev
spi = spidev.SpiDev()
# first zero is device, second zero is chip select (raspberry pi has 2 chip selects)
spi.open(0,0)
spi.max_speed_hz = 1000000
to_send = [0x01, 0x02, 0x03, 0x04]
assert spi.xfer(to_send)==to_send