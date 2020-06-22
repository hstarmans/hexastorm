# Run on raspberry pi
import spidev
spi = spidev.SpiDev()
# first zero is device, second zero is chip select (raspberry pi has 2 chip selects)
spi.open(0,0)
spi.max_speed_hz = 1000000
# all command are sent at least 2 times, so they are received at leas once
to_send = [0x01]*4+[0x02]*4+[0x03]*4
# they should be sent individually, so raise chip select after sent
received = []
for i in to_send:
    received+=spi.xfer([i])

def test(x):
    global received
    assert x in received
    received = list(filter((x).__ne__, received))
print(f"Received bytes {received}")
for i in [0,2,8]:
    test(i)
assert len(received) == 0
print("All required bytes mapped, test passed")