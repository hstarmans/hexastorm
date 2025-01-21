from machine import Pin, SoftSPI, SPI
from hexastorm.controller import Host

hst = Host(micropython=True)
hst.reset()
spi = SPI(2,
    baudrate=int(3e6),
    #polarity=1,
    phase=1,
    sck=Pin(12),
    mosi=Pin(13),
    miso=Pin(11))
spi.deinit()
spi.init()
flash_select = Pin(10, Pin.OUT)
flash_select.value(1)
fpga_select = Pin(9, Pin.OUT)

bts = [210, 110, 123, 202, 123, 134, 142, 99, 188, 187, 123, 203]
previous_byte = None

for i in range(100_000):
    response = bytearray([0]*len(bts))
    data = bytearray(bts)
    fpga_select.value(0)
    spi.write_readinto(data, response)
    fpga_select.value(1)
    try:
        assert data[:-2] == response[2:]
    except AssertionError:
        print(f"Test failed in loop {i}")
        break
