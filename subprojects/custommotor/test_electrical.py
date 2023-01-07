import unittest
from time import sleep, time_ns, time
import spidev
from copy import deepcopy
import csv
import subprocess

from gpiozero import LED
import numpy as np
import pandas as pd

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
            cls.platform.build(Driver(Firestarter(),
                                      top=True),
                               do_program=True,
                               verbose=True)
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
        #state = int.from_bytes(response[1:2], "big")
        #ticks = int.from_bytes(response[2:], "big")
        state = int.from_bytes(response, "big")
        return state

    def test_readfreq(self, delay=0):
        '''turns on the motor board and retrieves the rotor frequency

        Method runs for ever, can be interrupted with keyboard interrupt.
        '''
        lst = []
        start = time()
        starttime = 10
        totaltime = 60
        output = pd.DataFrame(columns=['time',
                                       'state'])
        print(f'Waiting {starttime} seconds to start measurement.')
        sleep(starttime)
        print("Starting measurement")
        try:
            while True:
                if (time()-start) >= totaltime:
                    print("Measurement finished")
                    output.to_csv('measurement.csv')
                    output = output[output['state'] != 0]
                    # six states cover 180 degrees
                    print((output.rename(columns={'time':'degrees'})
                                 .groupby(['state'])
                                 .count()/len(output)*180)
                          .assign(cumsum = lambda df: df['degrees'].cumsum())
                          .round())
                    break
                state = self.read_freq()
                #state = int(str(ticks)[:1])
                try:
                    dct = {'time': [time_ns()],
                           'state': [state]}
                    frame1 = pd.DataFrame(dct)
                    output = pd.concat([output, frame1],
                                       ignore_index=True)
                except ValueError as e:
                    print(e)
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
            print('Interrupted, exiting')


if __name__ == "__main__":
    unittest.main()
