from sys import platform

if platform == "linux": from smbus2 import SMBus

from hexastorm import board, core

class Scanhead:
    '''
    class used to control a scanhead flashed with binary from core
    '''
    device_nr = 1
    address = 0x28
    
    def __init__(self):
        if platform != "linux": raise Exception("OS not supported")
        self.bus = SMBus(self.device_nr)

    @property
    def laser_power(self):
        return self.bus.read_byte_data(self.address,0)
    
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
        self.bus.write_byte_data(self.address,0, val)

    def createbin(self):
        plat = board.Platform()
        spi_port = plat.request("spi")
        laser0 = plat.request("laser0")
        poly_pwm = plat.request("poly_pwm")
        poly_en = plat.request("poly_en")
        photodiode = plat.request("photodiode")
        spi_statemachine = core.Scanhead(spi_port, laser0, poly_pwm, poly_en, photodiode)
        plat.build(spi_statemachine, build_name = 'scanhead')