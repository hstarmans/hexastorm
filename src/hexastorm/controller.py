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

    def createbin(self):
        plat = board.Platform()
        hexacore = core.Scanhead(plat)
        import platform
        import os
        foldername = 'build'
        if os.path.isdir(foldername):
            raise Exception("build dir already exists, please remove")
        if 'arm' in platform.machine():
            # arm platform: use apio --pre-pack can't be passed as binary is not compiled with python
            print("Using apio extension with freq 50 MHz")
            plat.toolchain.nextpnr_build_template = [
                'apio raw "yosys -q -l {build_name}.rpt {build_name}.ys"',
                'apio raw "nextpnr-ice40 {pnr_pkg_opts} --pcf {build_name}.pcf --json {build_name}.json --asc {build_name}.txt --freq 50"',
                'apio raw "icepack {build_name}.txt {build_name}.bin"'
            ]
        plat.build(hexacore, build_name = 'scanhead')
        import subprocess
        proc = subprocess.Popen(['icoprog', '-p'], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        with open('build/scanhead.bin', 'rb') as bitstream:
            _, stderr = proc.communicate(input=bitstream.read())
        if stderr:  raise Exception("Not able to upload bitstream")
        import shutil
        shutil.rmtree('build')