""" raspberry pi test script

test script to run on the raspberry pi

There are problems with SPI. The select pin does not work as expected.
This is most likely due to spidev library. I remapped the cs pin:
dtoverlay=spi0-1cs,cs0_pin=18
Furthermore, xfer "silently" removes items from a list.
You should make a deepcopy
"""
import spidev
from gpiozero import LED, MCP3008

PIZERO = False
clock_pin = 11
mosi_pin = 10
miso_pin = 9
select_pin = 8

if PIZERO:
    dev = MCP3008(
        channel=0,
        mosi_pin=mosi_pin,
        miso_pin=miso_pin,
        select_pin=select_pin,
        clock_pin=clock_pin,
    )
else:
    spi = spidev.SpiDev()
    spi.open(0, 0)
    spi.mode = 1
    chip_select = LED(select_pin)
    chip_select.on()
    spi.max_speed_hz = round(1e6)
bts = [210, 222, 230]
previous_byte = None
for idx, byte in enumerate(bts):
    if PIZERO:
        byte_received = dev._spi.transfer([byte])
    else:
        chip_select.off()
        byte_received = spi.xfer([byte])[0]
        chip_select.on()
    if idx != 0:
        try:
            assert previous_byte == byte_received
        except AssertionError:
            print(previous_byte)
            print(byte_received)
            raise Exception("Test failed: not equal")
    previous_byte = byte
