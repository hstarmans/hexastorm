import os
import spidev
from time import sleep, time
import math

import numpy as np
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
    
    def __init__(self):
        '''
        if not connected to hardware implentation
        connect to a virtual model, see virtual test
        '''
        self.sh = hs.core.Scanhead(hs.board.Platform())
        # IC bus used to set power laser
        self.bus = SMBus(self.ic_dev_nr)
        # SPI to sent data to scanner
        self.spi = spidev.SpiDev()
        self.spi.open(0,0)
        self.spi.max_speed_hz = round(1E6)
        self.spi.cshigh = False
    
    @property
    def single_facet(self):
        '''
        return if system is in single facet mode
        '''
        return hs.core.Scanhead.VARIABLES['SINGLE_FACET']

    @single_facet.setter
    def single_facet(self, val):
        '''
        set system in single facet mode

        You need to run flash to push settings to head
        '''
        assert isinstance(val, bool)
        hs.core.Scanhead.VARIABLES['SINGLE_FACET'] = val


    @property
    def single_line(self):
        '''
        return if system is in single line mode
        '''
        return hs.core.Scanhead.VARIABLES['SINGLE_LINE']

    @single_line.setter
    def single_line(self, val):
        '''
        set system in single line mode

        You need to run flash to push settings to head
        '''
        assert isinstance(val, bool)
        hs.core.Scanhead.VARIABLES['SINGLE_LINE'] = val

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
        errors = [int(i) for i in list('{0:0b}'.format(byte&0b111111))]
        errors.reverse()
        return {'statebits': byte>>5, 'errorbits': errors}

    def status(self, byte=None):
        '''
        prints state machine and list of errors
        '''
        #TODO: this will not work if the machine is receiving
        if byte is None:
            state = self.spi.xfer([self.sh.COMMANDS.STATUS])[0]
        else:
            state = byte
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

    def forcewrite(self, data, maxtrials=100):
        state = self.get_state((self.spi.xfer([data]))[0])
        assert state['statebits'] in [self.sh.STATES.STOP, self.sh.STATES.START]
        trials = 0
        while (state['errorbits'][self.sh.ERRORS.MEMFULL] == 1):
            state = self.get_state((self.spi.xfer([data]))[0])
            trials += 1
            sleep(0.1)
            if trials>maxtrials:
                self.status()
                self.reset()
                raise Exception(f"More than {maxtrials} trials required to write to memory")

    def writeline(self, bitlst, bitorder = 'little'):
        '''
        writes bitlst to memory
        if bitlst is empty --> stop command is sent
        '''
        if len(bitlst) == 0:
            bytelst = [self.sh.INSTRUCTIONS.STOP]
        else:
            assert len(bitlst) == self.sh.BITSINSCANLINE
            assert max(bitlst) <= 1
            assert min(bitlst) >= 0
            bytelst = np.packbits(bitlst, bitorder=bitorder).tolist()
            bytelst = [self.sh.INSTRUCTIONS.SCAN] + bytelst
        bytelst.reverse()
        for _ in range(math.ceil(len(bytelst)/self.sh.CHUNKSIZE)):
            self.forcewrite(self.sh.COMMANDS.WRITE_L)
            for _ in range(self.sh.CHUNKSIZE): 
                try:
                    byte = bytelst.pop()
                except IndexError:
                    byte = 0    
                self.forcewrite(byte)

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
        plat = hs.board.Platform() #this object gets depleted, io objects get removed from list if requested
        self.sh = hs.core.Scanhead(plat)  #needs reinit to update settings
        if recompile: 
            if os.path.isdir('build'):
                plat.removebuild()
            plat.build(freq=50, core = self.sh, build_name = build_name)
        plat.upload(build_name)
        if removebuild:
            plat.removebuild()
        self.reset()  #TODO: you need to reset, why?!!