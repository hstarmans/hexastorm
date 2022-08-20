import unittest
from time import sleep, time_ns, time
import spidev
from copy import deepcopy
import csv

from gpiozero import LED
from platforms import Firestarter
from driver import Driver
from struct import unpack


class Base(unittest.TestCase):
    @classmethod
    def setUpClass(cls, build=True):
        platform = Firestarter()
        cls.reset_pin = LED(platform.reset_pin)
        cls.chip_select = LED(platform.chip_select)

        # SPI to sent data to scanner
        cls.spi = spidev.SpiDev()
        cls.spi.open(*platform.spi_dev)
        cls.spi.mode = 1
        cls.spi.max_speed_hz = round(1E6)
        if build:
            print("Building and programming board")
            platform = Firestarter()
            platform.build(Driver(Firestarter(), top=True),
                           do_program=True, verbose=True)
        else:
            cls.reset_pin.off()
            sleep(1)
            cls.reset_pin.on()
            sleep(1)
            print("Not programming board")

    def test_readfreq(self):
        with open(str(int(time()))+'.csv', 'w') as csvfile:
            while True:
                data = [1]*5  # random bytes
                self.chip_select.off()
                # spidev changes values passed to it
                datachanged = deepcopy(data)
                response = bytearray(self.spi.xfer(datachanged))
                self.chip_select.on()
                ticks = unpack(">I", bytearray(response[1:]))[0]
                writer = csv.writer(csvfile)
                writer.writerow([time_ns(), ticks])
                if ticks != 0:
                    freq = (12E6/(ticks*2))
                    print(f"Freq is {freq:.2f} and RPM is {freq*60:.2f}")
                else:
                    print(f"Invalid measured 0")
                sleep(1)
#     def test_sensor(self):
#         # je kunt de SDO pin veranderen vanuit FPGA
#         # je leest verschil met multimeter op pin
#         # je kunt ook de sensor pin veranderen van uit fpga
#         print('Reading sensor')
#         while True:
#             sleep(1)
#             print(self.sensor_pin0.value,
#                   self.sensor_pin1.value, 
#                   self.sensor_pin2.value)
#     def test_motor(self):
#         self.enable_pin.on()
#         while True:
#             sleep(1)
#             print(self.sensor_pin.value)


if __name__ == "__main__":
    unittest.main()
