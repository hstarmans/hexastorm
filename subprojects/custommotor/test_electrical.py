import unittest
from time import sleep, time_ns, time
import spidev
from copy import deepcopy
import csv
import subprocess

from gpiozero import LED
import numpy as np

from platforms import Firestarter
from driver import Driver
from struct import unpack


class Base(unittest.TestCase):
    @classmethod
    def setUpClass(cls, build=False):
        '''programs board

        if build is False FPGA is only reset
        and not flashed.
        '''
        cls.platform = Firestarter()
        cls.reset_pin = LED(cls.platform.reset_pin)
        cls.chip_select = LED(cls.platform.chip_select)

        # SPI to sent data to scanner
        cls.spi = spidev.SpiDev()
        cls.spi.open(*cls.platform.spi_dev)
        cls.spi.mode = 1
        cls.spi.max_speed_hz = round(1E6)
        if build:
            print("Building and programming board")
            cls.platform.build(Driver(Firestarter(), top=True),
                               do_program=True, verbose=True)
        else:
            cls.reset_pin.off()
            sleep(1)
            cls.reset_pin.on()
            sleep(1)
            print("Not programming board")

    def read_freq(self):
        '''reads rotor frequency in ticks via SPI''' 
        data = [1]*5  # random bytes
        self.chip_select.off()
        # spidev changes values passed to it
        datachanged = deepcopy(data)
        response = bytearray(self.spi.xfer(datachanged))
        self.chip_select.on()
        ticks = unpack(">I", bytearray(response[1:]))[0]
        if ticks != 0:
            freq = (13.56E6/ticks)
        else:
            freq = None
        return freq

    def test_readfreq(self, delay=1):
        '''turns on the motor board and retrieves the rotor frequency

        Method runs for ever, can be interrupted with keyboard interrupt.
        '''
        with open(str(int(time()))+'.csv', 'w') as csvfile:
            lst = []
            try:
                cntr = 0
                while True:
                    cntr += 1
                    if cntr == 120:
                        print("Starting measurement")
                    if cntr == 180:
                        print("Measurement finished")
                        break
                    freq = self.read_freq()
                    if cntr > 120:
                        if freq:
                            lst.append(freq)
                    writer = csv.writer(csvfile)
                    writer.writerow([time_ns(), freq])
                    if freq:
                        print(f"Freq is {freq:.2f} and RPM is {freq*60:.2f}")
                    else:
                        print(f"Invalid measured 0")
                    sleep(delay)
            except KeyboardInterrupt:
                pass
            finally:
                self.reset_pin.off()
                self.reset_pin.close()
                # gpiozero cleans up pins
                # this ensures pin is kept off
                command = subprocess.run(["raspi-gpio", "set",
                                          str(self.platform.reset_pin),
                                          "op", "dl"])
                ar = np.asarray(lst)
                print(f'Mean {np.mean(ar):.4f} and std {np.std(ar):.4f}')
                print('Interrupted, exiting')


if __name__ == "__main__":
    unittest.main()
