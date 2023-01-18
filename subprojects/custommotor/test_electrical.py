import unittest
from time import sleep, time_ns, time
import spidev
from copy import deepcopy
import csv
import subprocess

from gpiozero import LED
import numpy as np
import pandas as pd
import plotext as plt

from platforms import Firestarter
from driver import Driver
from struct import unpack


class Base(unittest.TestCase):
    @classmethod
    def setUpClass(cls, word='hallfilter', build=True):
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
        cls.word = word
        cls.divider = 800
        if build:
            print("Building and programming board")
            cls.platform.build(Driver(Firestarter(),
                                      word=word,
                                      divider=cls.divider,
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
        response = int.from_bytes(response, "big")
        if (self.word == 'cycletime') & (response != 0):
            # you measure 180 degrees
            response = round((12E6/(response*2)*60))
        return response

    def finish(self, output):
        print("Measurement finished")
        output.to_csv('measurement.csv')
        if self.word == 'hallfilter':
            output = output[output['word'] != 0]
            # six states cover 180 degrees
            print((output.rename(columns={'time':'degrees'})
                         .groupby(['word'])
                         .count()/len(output)*180)
                  .assign(cumsum = lambda df: df['degrees'].cumsum())
                  .round())
        elif self.word == 'cycletime':
            print(output[['word']].describe())
        elif self.word == 'angle':
            #print(output[['word']].describe())
            #print(output['word'].unique())
            bins = [0, 30, 60, 90, 120, 150, 180]
            labels = ['0', '30', '60', '90', '120', '150']
            output['hall'] = pd.cut(x=output['word'],
                                    bins=bins,
                                    labels=labels,
                                    include_lowest=True)
            print(output.hall.sort_values().value_counts()/len(output))
            #plt.hist(output['word'].tolist(), 6, label = "distribution")
            #plt.title("Histogram Plot")
            #plt.show()

    def test_readfreq(self, delay=0):
        '''turns on the motor board and retrieves the rotor frequency

        Method runs for ever, can be interrupted with keyboard interrupt.
        '''
        lst = []
        start = time()
        starttime = 10
        totaltime = 120
        output = pd.DataFrame(columns=['time',
                                       'word'])
        print(f'Waiting {starttime} seconds to start measurement.')
        sleep(starttime)
        print("Starting measurement")
        try:
            while True:
                if (time()-start) >= totaltime:
                    self.finish(output)
                    break
                word = self.read_freq()
                #state = int(str(ticks)[:1])
                try:
                    dct = {'time': [time_ns()],
                           'word': [word]}
                    frame1 = pd.DataFrame(dct)
                    output = pd.concat([output, frame1],
                                       ignore_index=True)
                    if (self.word in ['cycletime',
                                      'statecounter',
                                      'anglecounter']):
                        print(word)
                        sleep(1)
                except ValueError as e:
                    print(e)
        except KeyboardInterrupt:
            self.finish(output)
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
