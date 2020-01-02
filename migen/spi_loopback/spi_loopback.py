"""
    spi_slave.py
    Creates spi slave with loopback 
    Tested with icezero board
    
    Rik Starmans
"""

from litex.soc.cores.spi import SPISlave
# quick solution
import sys
sys.path.append('..')
import icezero as board

from migen import *

class MyTopModule(Module):
    def __init__(self, spi_port, width):
        my_SpiSlave = SPISlave(spi_port, width)
        self.submodules += my_SpiSlave
        loopback = my_SpiSlave.loopback
        self.comb += loopback.eq(1) 


plat = board.Platform()
spi_port = plat.request("spi")
#my_SpiSlave = MyTopModule(spi_port, 8)
my_SpiSlave = SPISlave(spi_port, 8)
my_SpiSlave.comb += my_SpiSlave.loopback.eq(1)

plat.build(my_SpiSlave, build_name = 'spi_loopback')









