import os
from time import sleep, time
import math

import spidev
import numpy as np
from gpiozero import LED
from smbus2 import SMBus

from hexastorm.core import Scanhead
import hexastorm.board as board

class Machine:
    '''
    class used to control a laser scanner
    '''
    ic_dev_nr = 1
    ic_address = 0x28
    
    def __init__(self):
        self.sh = Scanhead(board.Platform())
        # IC bus used to set power laser
        self.bus = SMBus(self.ic_dev_nr)
        # SPI to sent data to scanner
        self.spi = spidev.SpiDev()
        self.spi.open(0,0)
        self.spi.max_speed_hz = round(1E6)
        self.spi.cshigh = False
        self.STABLE_TIME = round(Scanhead.VARIABLES['SPINUP_TIME']
                                    + Scanhead.VARIABLES['STABLE_TIME']+2)

    @property
    def single_facet(self):
        'true if system is in single facet mode'
        return Scanhead.VARIABLES['SINGLE_FACET']

    @single_facet.setter
    def single_facet(self, val):
        '''set system in single facet mode

        You need to run flash to push settings to head
        '''
        assert isinstance(val, bool)
        Scanhead.VARIABLES['SINGLE_FACET'] = val


    @property
    def single_line(self):
        'true if system is in single line mode'
        return Scanhead.VARIABLES['SINGLE_LINE']

    @single_line.setter
    def single_line(self, val):
        '''set system in single line mode

        You need to run flash to push settings to head
        '''
        assert isinstance(val, bool)
        Scanhead.VARIABLES['SINGLE_LINE'] = val

    @property
    def laser_power(self):
        'return laser power in range [0-255]'
        return self.bus.read_byte_data(self.ic_address, 0)

    @laser_power.setter
    def laser_power(self, val):
        '''
        set the maximum laser current of driver chip to given value in range [0-255]
        This does not turn on or off the laser. 
        
        Laser will be set to this current if pulsed.
        The laser power can be changed in two ways.
        First by using one or two channels. Second by setting a value between
        0-255 at the laser driver chip.
        '''
        if val < 0 or val > 255: raise Exception('Invalid laser power')
        self.bus.write_byte_data(self.ic_address, 0, val)

    def bytetostate(self, byte=None):
        'seperate the state and error bits from a byte'
        if byte is None: byte = self.spi.xfer([self.sh.COMMANDS.STATUS])[0]
        errors = [int(i) for i in list('{0:0b}'.format(byte&0b11111))]
        errors.reverse()
        # pad the list to 5
        for _ in range(len(errors), 5): errors.append(0)
        return {'state': byte>>5, 'errorbits': errors}

    def statetobyte(self, errors=[], state=Scanhead.STATES.STOP):
        'create byte correspdonding to a list of errors and a certain state'
        errorstate = 0
        for error in errors: errorstate += pow(2, error)
        val = errorstate + (state<<5)
        return val

    def status(self, byte=None, verbose=True):
        '''prints state machine and list of errors for given byte
        if verbose is True

        returns the machine state and errors as string
        '''
        #TODO: this will not work if the machine is receiving
        bytedict = self.bytetostate(byte)
        state, errors = bytedict['state'], bytedict['errorbits']
        if state == 255: raise Exception("Check reset pin is high and binary is correct")
        error_string = 'None'
        if max(errors)>0:
            error_string = ''
            for idx, val in enumerate(errors):
                if val>0:
                    error = list(self.sh.ERRORS._asdict())[idx]
                    error_string += error + ' '
        machinestate = list(self.sh.STATES._asdict())[state]
        if verbose:
            print(f"The machine state is {machinestate}")
            print(f"The errors are {error_string}")
        return machinestate, error_string

    def start(self):
        'start scanhead'
        self.busywrite(self.sh.COMMANDS.START)

    def stop(self):
        'disables scanhead'
        self.busywrite(self.sh.COMMANDS.STOP)

    def test_laser(self):
        'enable laser'
        self.busywrite(self.sh.COMMANDS.LASERTEST)

    def test_line(self):
        'enable laser and motor which creates line'
        self.busywrite(self.sh.COMMANDS.LINETEST)

    def test_motor(self):
        'enable motor'
        self.busywrite(self.sh.COMMANDS.MOTORTEST)

    def reset(self, pin=26):
        'reset the chip by raising and lowering the reset pin'
        reset_pin = LED(pin)
        reset_pin.off()
        sleep(1)
        reset_pin.on()
        sleep(1)

    def busywrite(self, data, maxtrials=1E6):
        byte = (self.spi.xfer([data]))[0]
        state = self.bytetostate(byte)
        trials = 0
        #TODO: NOTSTABLE can also mean busy --> still needs fix
        while (state['errorbits'][self.sh.ERRORS.NOTSTABLE] == 1):
            byte = (self.spi.xfer([data]))[0]
            state = self.bytetostate(byte)
            trials += 1
            if trials>maxtrials:
                self.status()
                self.reset()
                raise Exception(f"More than {maxtrials} trials required to write to memory")
        return byte


    def forcewrite(self, data, maxtrials=1E6):
        byte = (self.spi.xfer([data]))[0]
        state = self.bytetostate(byte)
        trials = 0
        # NOTE: invalids should be detected
        #      this was an issue with an older version of the code
        if (state['errorbits'][self.sh.ERRORS.INVALID] == 1):
                raise Exception("INVALID DETECTED")
        while (state['errorbits'][self.sh.ERRORS.MEMFULL] == 1)|(state['errorbits'][self.sh.ERRORS.NOTSTABLE] == 1):
            byte = (self.spi.xfer([data]))[0]
            state = self.bytetostate(byte)
            trials += 1
            if (state['errorbits'][self.sh.ERRORS.INVALID] == 1):
                self.status()
                self.reset()
                raise Exception("INVALID DETECTED")
            if trials>maxtrials:
                self.status()
                self.reset()
                raise Exception(f"More than {maxtrials} trials required to write to memory")
    
    def bittobytelist(self, bitlst, bitorder = 'little'):
        '''converts bitlst to bytelst
        
        if bytelst is empty stop command is sent
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
        return bytelst

    def genwritebytes(self, bytelst):
        '''writes bytelst to memory

        generator notation to facilitate sharing function with virtual object
        '''
        for _ in range(math.ceil(len(bytelst)/self.sh.CHUNKSIZE)):
            yield self.sh.COMMANDS.WRITE_L
            for _ in range(self.sh.CHUNKSIZE): 
                try:
                    yield bytelst.pop()
                except IndexError:
                    yield 0

    def writeline(self, bitlst, bitorder = 'little'):
        '''writes bitlst to memory
        
        if bitlst is empty --> stop command is sent
        '''
        bytelst = self.bittobytelist(bitlst)
        for byte in self.genwritebytes(bytelst): self.forcewrite(byte)

    def test_photodiode(self):
        '''enable motor, laser and disable if photodiode is triggered

        returns False if succesfull
        '''
        self.busywrite(self.sh.COMMANDS.PHOTODIODETEST)
        sleep(2)
        res = True
        if self.bytetostate()['state']!=self.sh.STATES.STOP:
            print("Test failed, stopping")
            self.stop()
        else:
            res = False
            print("Test succeeded")
        return res

    def flash(self, recompile=False, removebuild=False):
        build_name = 'scanhead'
        plat = board.Platform()   # this object gets depleted, io objects get removed from list if requested
        self.sh = Scanhead(plat)  # needs reinit to update settings
        if recompile:
            if os.path.isdir('build'):
                plat.removebuild()
            plat.build(freq=50, core = self.sh,
                       build_name = build_name)
        plat.upload(build_name)
        if removebuild:
            plat.removebuild()
        self.reset()  #TODO: you need to reset, why?!!