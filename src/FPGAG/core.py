from collections import namedtuple

from nmigen import Signal, Cat, Elaboratable
from nmigen import Memory, Module
from luna.gateware.interface.spi import SPICommandInterface
from luna.gateware.memory import TransactionalizedFIFO

# NOTE: following doesnt work due to bug in pylint https://github.com/PyCQA/pylint/issues/3876
# def customnamedtuple(typename, field_names) -> namedtuple:
#    return namedtuple(typename, field_names,
#                      defaults=range(len(field_names)))

STATES = namedtuple('STATES', ['START', 'STOP'], defaults=range(2))()
COMMANDS = namedtuple('COMMANDS', ['EMPTY', 'FILLED', 'EXIT', 'ABORT'],
                      defaults=range(4))()
ERRORS = namedtuple('ERRORS', ['MEMFULL', 'MEMREAD', 'INVALID'],
                    defaults=range(3))()
CHUNKSIZE = 15  # first chunk is command
# one block is 4K bits, there are 32 blocks (officially 20 in HX4K)
MEMWIDTH = 16
MEMDEPTH = 256
VARIABLES = {'CRYSTAL_HZ': 50E6}


class Core(Elaboratable):
    """ FPGA core for Beage G. """

    def __init__(self):
        # Base ourselves around an SPI command interface.
        self.interface = SPICommandInterface(clock_phase=1)

    def elaborate(self, platform):
        m = Module()
        m.submodules.fifo = fifo = TransactionalizedFIFO(width=MEMWIDTH,
                                                         depth=MEMDEPTH)
        # begin met ontvangen commdos en schrijf naar buffer
        # schrijf hiervoor een test