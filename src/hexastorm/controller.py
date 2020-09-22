import os

from hexastorm import board, core

class Scanhead:
    '''
    class used to control a scanhead flashed with binary from core
    '''
    ic_dev_nr = 1
    ic_address = 0x28
    
    def __init__(self, virtual = True):
        '''
        virtual: false scanhead is actually used
        '''
        self.virtual = virtual
        if virtual:
            self._laserpower = 128
        else:
            from smbus2 import SMBus
            self.bus = SMBus(self.ic_dev_nr)

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

    def createbin(self, recompile=False, removebuild=False):
        plat = board.Platform()
        hexacore = core.Scanhead(plat)
        if not recompile and not os.path.isdir('build'):
            recompile = True
        if recompile:
            plat.build(freq=50, core=hexacore, build_name = 'scanhead')
        if removebuild: plat.removebuild()