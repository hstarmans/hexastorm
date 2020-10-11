import os
import spidev
from time import sleep
import math

from gpiozero import LED
from smbus2 import SMBus

import hexastorm.controller
import hexastorm.core
import hexastorm.board
import hexastorm as hs

class Machine:
    '''
    class used to control a laser scanner
    '''
    ic_dev_nr = 1
    ic_address = 0x28
    
    def __init__(self, virtual = False):
        '''
        virtual: false scanhead is actually used
        '''
        self.plat = hs.board.Platform()
        self.sh = hs.core.Scanhead(self.plat)
        # IC bus used to set power laser
        self.bus = SMBus(self.ic_dev_nr)
        # SPI to sent data to scanner
        self.spi = spidev.SpiDev()
        self.spi.open(0,0)
        self.spi.max_speed_hz = round(1E6)
        self.spi.cshigh = False

    @property
    def single_line(self):
        '''
        return if system is in single line mode
        '''
        return self.sh.VARIABLES['SINGLE_LINE']

    @single_line.setter
    def single_line(self, val):
        '''
        set system in single line mode
        '''
        assert isinstance(val, bool)
        self.sh.VARIABLES['SINGLE_LINE'] = val
        self.flash(recompile=True, removebuild=True)

    @property
    def laser_power(self):
        return self.bus.read_byte_data(self.ic_address, 0)

    @laser_power.setter
    def laser_power(self, val):
        '''
        set laser power to given value in range [0-255]
        for the laser driver chip. This does not turn on or off the laser.
        
        The laser power can be changed in two ways.
        First by using one or two channels. Second by settings a value between
        0-255 at the laser driver chip.
        '''
        if val < 0 or val > 255: raise Exception('Invalid laser power')
        self.bus.write_byte_data(self.ic_address, 0, val)

    def get_state(self, byte=None):
        '''
        grabs state and error bits
        '''
        if byte is None: byte = self.spi.xfer([self.sh.COMMANDS.STATUS])[0]
        return {'statebits': byte>>5, 'errorbits': byte&0b11111}

    def status(self):
        '''
        prints state machine and list of errors
        '''
        #TODO: this will not work if the machine is receiving
        state = self.spi.xfer([self.sh.COMMANDS.STATUS])[0]
        if state == 255:
            print("Error; check reset pin is high and binary is correct")
            return
        errors = [int(i) for i in list('{0:0b}'.format(state&0b11111))]
        errors.reverse()
        if max(errors)>0:
            print("Detected errors; ", end='')
            for idx, val in enumerate(errors):
                if val>0:
                    error = list(self.sh.ERRORS._asdict())[idx]
                    print(error, end=' ')
            print() # to endline
        machinestate = list(self.sh.STATES._asdict())[state>>5]
        print(f"The machine state is {machinestate}")

    def start(self):
        '''
        start scanhead
        '''
        self.spi.xfer([self.sh.COMMANDS.START])

    def stop(self):
        '''
        disables scanhead
        '''
        self.spi.xfer([self.sh.COMMANDS.STOP])

    def test_laser(self):
        '''
        enable laser
        '''
        self.spi.xfer([self.sh.COMMANDS.LASERTEST])

    def test_line(self):
        '''
        enable laser and motor and create line
        '''
        self.spi.xfer([self.sh.COMMANDS.LINETEST])

    def test_motor(self):
        '''
        enable motor
        '''
        self.spi.xfer([self.sh.COMMANDS.MOTORTEST])

    def state(self, errors=[], state=hs.core.Scanhead.STATES.STOP):
        ''' 
        given a list of errors and a certain state
        this function returns the state encoding
        '''
        errorstate = 0
        for error in errors: errorstate += pow(2, error)
        val = errorstate + (state<<5)
        return [val]

    def reset(self, pin=26):
        '''
        reset the chip by raising and lowering the reset pin
        '''
        reset_pin = LED(pin)
        reset_pin.off()
        sleep(1)
        reset_pin.on()
        sleep(1)


    def writeline(self, bytelst, lastline = False):
        '''
        writes bytelst to memory
          if bytelst is empty --> last line command is given
         preceded by scan command

        return: the bytes it wasn't able to write if memory gets full
        '''
        assert len(bytelst) == self.sh.BITSINSCANLINE
        assert max(bytelst) <= 255
        assert min(bytelst) >= 0
        bytelst = [self.sh.INSTRUCTIONS.STOP] + bytelst if lastline else [self.sh.INSTRUCTIONS.SCAN] + bytelst
        for _ in range(math.ceil(len(bytelst)/self.sh.MEMWIDTH)):
            state = self.get_state(self.spi.xfer([self.sh.COMMANDS.WRITE_L])[0])
            assert state['statebits'] in [self.sh.STATES.STOP, self.sh.STATES.START]
            if state['errorbits'] == pow(2, self.sh.ERRORS.MEMFULL): return bytelst
            for _ in range(self.sh.CHUNKSIZE): 
                try:
                    state = self.spi.xfer([bytelst.pop()])
                except IndexError:
                    self.spi.xfer([0])
        return bytelst


    def test_photodiode(self):
        '''
        enable motor, laser and disable if photodiode is triggered

        returns False if succesfull and True if unsuccesfull
        '''
        self.spi.xfer([self.sh.COMMANDS.PHOTODIODETEST])
        sleep(2)
        res = True
        if self.get_state()['statebits']!=self.sh.STATES.STOP:
            print("Test failed, stopping")
            self.stop()
        else:
            res = False
            print("Test succeeded")
        return res

    def flash(self, recompile=False, removebuild=False):
        build_name = 'scanhead'
        if recompile: 
            if os.path.isdir('build'):
                self.plat.removebuild()
            self.plat.build(freq=50, core = self.sh, build_name = build_name)
        self.plat.upload(build_name)
        if removebuild:
            self.plat.removebuild()