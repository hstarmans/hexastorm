# Run on raspberry pi
import spidev
spi = spidev.SpiDev()
# first zero is device, second zero is chip select (raspberry pi has 2 chip selects)
spi.open(0,0)
spi.max_speed_hz = 1000000
to_send = [0x01]
count = spi.xfer(to_send)[0]
print("The current count is {}".format(count))
assert spi.xfer(to_send) == [count+1]
assert spi.xfer(to_send) == [count+2]  
assert spi.xfer(to_send) == [count+3]  