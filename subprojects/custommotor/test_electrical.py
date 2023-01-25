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
    def setUpClass(cls,
                   word='PIcontrol',
                   PIcontrol=True,
                   build=True):
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
                                      PIcontrol=PIcontrol,
                                      top=True),
                               do_program=True,
                               verbose=True)
        else:
            cls.reset_pin.off()
            sleep(1)
            cls.reset_pin.on()
            sleep(1)
            print("Not programming board")

    def read_word(self):
        '''reads word sent over via SPI'''
        data = [1]*5  # random bytes
        self.chip_select.off()
        # spidev changes values passed to it
        datachanged = deepcopy(data)
        # first byte back is commmand
        response = bytearray(self.spi.xfer(datachanged))[1:]
        self.chip_select.on()
        clock = int(self.platform.clks[self.platform.hfosc_div]*1E6)
        if (self.word == 'cycletime') & (response != 0):
            response = int.from_bytes(response, "big")
            # you measure 180 degrees
            response = round((clock/(response*2)*60))
        elif (self.word == 'PIcontrol'):
            degreecnt = int.from_bytes(response[2:], "big", signed=False)
            if degreecnt != 0:
                speed = (clock/(degreecnt*180*2)*60)
            else:
                speed = 0
            delay = int.from_bytes(response[:2], "big", signed=True)
            response = [degreecnt, delay]
        elif (self.word == 'anglecounter'):
            degreecnt = int.from_bytes(response, "big")
            if degreecnt != 0:
                response = (clock/(degreecnt*180*2)*60)
            else:
                response = 0
        else:
            response = int.from_bytes(response, "big")
        if not isinstance(response,
                          list):
            return [response]
        else:
            return response

    def finish(self, output):
        print(f"Measurement finished in mode {self.word}")
        output.to_csv('measurement.csv')
        if self.word == 'hallfilter':
            output = output[output['word'] != 0]
            # six states cover 180 degrees
            print((output.rename(columns={'time':'degrees'})
                         .groupby(['word_0'])
                         .count()/len(output)*180)
                  .assign(cumsum = lambda df: df['degrees'].cumsum())
                  .round())
        elif self.word == 'cycletime':
            print(output[['word_0']].describe())
        elif self.word == 'angle':
            #print(output[['word']].describe())
            #print(output['word'].unique())
            bins = [0, 30, 60, 90, 120, 150, 180]
            labels = ['0', '30', '60', '90', '120', '150']
            output['hall'] = pd.cut(x=output['word_0'],
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
        start = time()
        starttime = 10
        totaltime = 60
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
                words = self.read_word()
                #state = int(str(ticks)[:1])
                try:
                    dct = {'time': [time_ns()]}
                    for idx, word in enumerate(words):
                        dct[f'word_{idx}'] = word
                    frame1 = pd.DataFrame(dct)
                    output = pd.concat([output, frame1],
                                       ignore_index=True)
                    if (self.word in ['cycletime',
                                      'statecounter',
                                      'PIcontrol',
                                      'anglecounter']):
                        for word in words:
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
