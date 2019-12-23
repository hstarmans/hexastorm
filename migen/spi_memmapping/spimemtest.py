# Run on raspberry pi
import spidev
spi = spidev.SpiDev()
# first zero is device, second zero is chip select (raspberry pi has 2 chip selects)
spi.open(0,0)
spi.max_speed_hz = 1000000
write_mem = [0x01]
get_mem = [0x02]
change_address = [0x03]
address1 = [0x01]
address2 = [0x02]
data1 = [0x03]
data2 = [0x05]
# change addres to 1
assert spi.xfer([0x03]) == [0]
assert spi.xfer([0x01]) == [0x03]
# read data at 1
assert spi.xfer([0x02]) == [0]
print("Data at address 1 {}".format(spi.xfer(get_mem)))
# write data to 1 
assert spi.xfer([0x01]) == [0]
assert spi.xfer([0x03]) == [0x01]
# change addres to 2
assert spi.xfer([0x03]) == [0]
assert spi.xfer([0x02]) == [0x03]
# read data at 2
assert spi.xfer([0x02]) == [0]
print("Data at address 2 {}".format(spi.xfer([0x02])))
# write data to 2
assert spi.xfer([0x01]) == [0]
assert spi.xfer([0x05]) == [0x01]
# read data at 2
assert spi.xfer([0x02]) == [0]
print("Data at address 22 {}".format(spi.xfer([0x02])))
# change addres to 1
assert spi.xfer([0x03]) == [0]
assert spi.xfer([0x01]) == [0x03]
# request data
assert spi.xfer([0x02]) == [0]
assert spi.xfer([0x02]) == [0x03]