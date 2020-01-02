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
# read from address 0, check equal to 10
assert spi.xfer([2]) == [0]
assert spi.xfer([2]) == [10]
# change address to 1
assert spi.xfer([3]) == [0]
assert spi.xfer([1]) == [3]
# read from address 1, check equal to 10
assert spi.xfer([2]) == [0]
assert spi.xfer([2]) == [20]
# write 5 to address 1 
assert spi.xfer([1]) == [0]
assert spi.xfer([5]) == [1]
# change address to 0
assert spi.xfer([3]) == [0]
assert spi.xfer([0]) == [3]
# read data at 0, check equal to 10
assert spi.xfer([2]) == [0]
assert spi.xfer([2]) == [10]
# change address to 1
assert spi.xfer([3]) == [0]
assert spi.xfer([1]) == [3]
# read from address 1, check equal to 5
assert spi.xfer([2]) == [0]
assert spi.xfer([2]) == [5]