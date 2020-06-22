import sys

import pigpio
import spidev

#
chip_select = 14

pi = pigpio.pi()
if not pi.connected:
    print("Can't connect to pigpio Daemon")
    sys.exit(1)

spi = spidev.SpiDev()
# first zero is device, second zero is chip select (raspberry pi has 2 chip selects)
# at the moment a custom chip select is used which is always enabled
pi.write(chip_select, 0)
spi.open(0,0)
spi.max_speed_hz = 1000000
to_send = [0x01]
pi.write(chip_select, 0)
count = spi.xfer(to_send)[0]
pi.write(chip_select, 1)
print("The current count is {}".format(count))
try:
    pi.write(chip_select, 0)
    assert spi.xfer(to_send) == [count+1]
    pi.write(chip_select, 1)
    pi.write(chip_select, 0)
    assert spi.xfer(to_send) == [count+2]
    pi.write(chip_select, 1)
    pi.write(chip_select, 0)
    assert spi.xfer(to_send) == [count+3]
    pi.write(chip_select, 1)
    print("Test succeeded")
except AssertionError:
    print("Test failed")
pi.write(chip_select, 1)  