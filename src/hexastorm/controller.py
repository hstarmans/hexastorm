import os
import spidev
from time import sleep

from gpiozero import LED

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
        self.virtual = virtual
        if virtual:
            self._laserpower = 128
        else:
            from smbus2 import SMBus
            # IC bus used to set power laser
            self.bus = SMBus(self.ic_dev_nr)
            # SPI to sent data to scanner
            self.spi = spidev.SpiDev()
            self.spi.open(0,0)
            self.spi.max_speed_hz = round(1E6)
            self.spi.cshigh = False

    @property
    def laser_power(self):
        if self.virtual: 
            return self._laserpower
        else:
            return self.bus.read_byte_data(self.ic_address,0)

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
        if self.virtual:
            self._laserpower = val
        else:
            self.bus.write_byte_data(self.ic_address, 0, val)

    def get_state(self):
        return self.spi.xfer([hs.core.Scanhead.COMMANDS.STATUS])[0]>>5

    def status(self):
        '''
        prints state machine and list of errors
        '''
        #TODO: this will not work if the machine is receiving
        state = self.spi.xfer([hs.core.Scanhead.COMMANDS.STATUS])[0]
        if state == 255:
            print("Error; check reset pin is high and binary is correct")
            return
        errors = [int(i) for i in list('{0:0b}'.format(state&0b11111))]
        errors.reverse()
        if max(errors)>0:
            print("Detected errors; ", end='')
            for idx, val in enumerate(errors):
                if val>0:
                    error = list(hs.core.Scanhead.ERRORS._asdict())[idx]
                    print(error, end=' ')
            print() # to endline
        machinestate = list(hs.core.Scanhead.STATES._asdict())[state>>5]
        print(f"The machine state is {machinestate}")

    def start(self):
        '''
        start scanhead
        '''
        self.spi.xfer([hs.core.Scanhead.COMMANDS.START])

    def stop(self):
        '''
        disables scanhead
        '''
        self.spi.xfer([hs.core.Scanhead.COMMANDS.STOP])

    def test_laser(self):
        '''
        enable laser
        '''
        self.spi.xfer([hs.core.Scanhead.COMMANDS.LASERTEST])

    def test_line(self):
        '''
        enable laser and motor and create line
        '''
        self.spi.xfer([hs.core.Scanhead.COMMANDS.LINETEST])

    def test_motor(self):
        '''
        enable motor
        '''
        self.spi.xfer([hs.core.Scanhead.COMMANDS.MOTORTEST])

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

    def test_photodiode(self):
        '''
        enable motor, laser and disable if photodiode is triggered

        returns False if succesfull and True if unsuccesfull
        '''
        self.spi.xfer([hs.core.Scanhead.COMMANDS.PHOTODIODETEST])
        sleep(2)
        res = True
        if self.get_state()!=hs.core.Scanhead.STATES.STOP:
            print("Test failed, stopping")
            self.stop()
        else:
            res = False
            print("Test succeeded")
        return res

    def flash(self, recompile=False, removebuild=False):
        build_name = 'scanhead'
        plat = hs.board.Platform()
        if recompile: 
            if os.path.isdir('build'):
                plat.removebuild()
            plat.build(freq=50, core = hs.core.Scanhead(plat), build_name = build_name)
        plat.upload(build_name)
        if removebuild:
            plat.removebuild()