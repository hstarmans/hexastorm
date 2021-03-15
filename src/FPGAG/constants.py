""" Constants

Settings of the implementation are saved in board.by and constants.py.
Constants are more related to the actual implementation on the FPGA.
"""
from collections import namedtuple

COMMANDS = namedtuple('COMMANDS', ['EMPTY', 'WRITE', 'STATUS',
                                   'START', 'STOP'],
                      defaults=range(5))()
INSTRUCTIONS = namedtuple('INSTRUCTIONS', ['MOVE'], defaults=[1])()
STATE = namedtuple('STATE', ['FULL', 'DISPATCHERROR'], defaults=range(2))()
# one block is 4K there are 32 blocks (officially 20 in HX4K)
# max of 1 block is 16*256 but if you use a larger memwidth you seem to
# use more blocks so this might work
# MEMDEPTH = 256  memdepth is defined on the board so it can be changed
COMMAND_BYTES = 1
WORD_BYTES = 8
MEMWIDTH = WORD_BYTES*8
FREQ = 1E6  # motor move interpolation freq in Hz
MOVE_INSTRUCTION = {'INSTRUCTION': 1, 'TICKS': 7}
DEGREE = 3  # only third degree polynomal

# these numbers must be tested with minimum
# jerk and maximum velocity move
BIT_SHIFT = 40
MOVE_TICKS = 10_000

# TODO: move this to board
VARIABLES = {'CRYSTAL_HZ': 50E6}


def getbytesinmove(motors):
    bytesingcode = (sum(MOVE_INSTRUCTION.values())
                    + motors*DEGREE*WORD_BYTES)
    bytesingcode += bytesingcode % WORD_BYTES
    return bytesingcode


def getmovedct(motors):
    dct = MOVE_INSTRUCTION
    for i in range(motors):
        for j in range(DEGREE):
            dct.update({f'C{i}{j}': 8})

# NOTE: following doesnt work due to bug
# in pylint https://github.com/PyCQA/pylint/issues/3876
# def customnamedtuple(typename, field_names) -> namedtuple:
#    return namedtuple(typename, field_names,
#                      defaults=range(len(field_names)))
