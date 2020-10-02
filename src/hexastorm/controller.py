import os
import spidev
from time import sleep

from gpiozero import LED

from hexastorm import board
from hexastorm.core import Scanhead

class Machine:
    '''
    class used to control a scanhead flashed with binary from core
    '''
    ic_dev_nr = 1
    ic_address = 0x28
    reset_gpio = 26
    
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
            self.reset_pin = LED(self.reset_gpio)

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
        return self.spi.xfer([Scanhead.COMMANDS.STATUS])[0]>>5

    def status(self):
        '''
        prints state machine and list of errors
        '''
        #TODO: this will not work if the machine is receiving
        state = self.spi.xfer([Scanhead.COMMANDS.STATUS])[0]
        errors = [int(i) for i in list('{0:0b}'.format(state&0b11111))]
        if max(errors)>0:
            print("Detected errors; ", end='')
            for idx, val in enumerate(errors):
                error = list(Scanhead.ERRORS._asdict())[idx]
                if val>0: print(error, end='')
            print() # to endline
        machinestate = list(Scanhead.STATES._asdict())[state>>5]
        print(f"The machine state is {machinestate}")

    def stop(self):
        '''
        disables scanhead
        '''
        self.spi.xfer([Scanhead.COMMANDS.STOP])

    def test_laser(self):
        '''
        enable laser
        '''
        self.spi.xfer([Scanhead.COMMANDS.LASERTEST])

    def test_line(self):
        '''
        enable laser and motor and create line
        '''
        self.spi.xfer([Scanhead.COMMANDS.LINETEST])

    def test_motor(self):
        '''
        enable motor
        '''
        self.spi.xfer([Scanhead.COMMANDS.MOTORTEST])

    def state(self, errors=[], state=Scanhead.STATES.STOP):
        ''' 
        given a list of errors and a certain state
        this function returns the state encoding
        '''
        errorstate = 0
        for error in errors: errorstate += pow(2, error)
        val = errorstate + (state<<5)
        return [val]

    def reset(self):
        '''
        reset the chip by raising and lowering the reset pin
        '''
        self.reset_pin.off()
        sleep(0.1)
        self.reset_pin.on()


    def test_photodiode(self):
        '''
        enable motor, laser and disable if photodiode is triggered

        returns False if succesfull and True if unsuccesfull
        '''
        self.spi.xfer([Scanhead.COMMANDS.PHOTODIODETEST])
        sleep(2)
        res = True
        if self.get_state()!=Scanhead.STATES.STOP:
            print("Test failed, stopping")
            self.stop()
        else:
            res = False
            print("Test succeeded")
        return res

    def flash(self, recompile=False, removebuild=False):
        plat = board.Platform()
        hexacore = Scanhead(plat)
        build_name = 'scanhead'
        if not recompile and not os.path.isdir('build'): recompile = True
        if recompile: plat.build(freq=50, core=hexacore, build_name = build_name)
        plat.upload(build_name)
        if removebuild: plat.removebuild()