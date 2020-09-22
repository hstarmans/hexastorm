import spidev

spi = spidev.SpiDev()
# first zero is device, second zero is chip select (raspberry pi has 2 chip selects)
spi.open(0,0)
spi.max_speed_hz = 1000000
to_send = [0x01]
count = spi.xfer(to_send)[0]
print("The current count is {}".format(count))
for i in range(100):
    oldcount = count%256
    count = spi.xfer(to_send)[0]
    try:
        assert count == (oldcount+2)%256
    except AssertionError:
        print(f"Expected count {oldcount+1} but got {count}")
        break