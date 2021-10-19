""" Constants

Settings of the implementation are saved in board.by and constants.py.
Constants are more related to the actual implementation on the FPGA.
"""
from collections import namedtuple
from math import ceil

COMMANDS = namedtuple('COMMANDS', ['EMPTY', 'WRITE', 'READ', 'POSITION',
                                   'START', 'STOP'],
                      defaults=range(6))()
INSTRUCTIONS = namedtuple('INSTRUCTIONS', ['MOVE', 'WRITEPIN', 'SCANLINE',
                          'LASTSCANLINE'],
                          defaults=range(1, 5))()
STATE = namedtuple('STATE', ['FULL', 'PARSING', 'ERROR'],
                   defaults=range(3))()

COMMAND_BYTES = 1
WORD_BYTES = 8
MEMWIDTH = WORD_BYTES*8
FREQ = 1E6  # motor move interpolation freq in Hz
MOVE_TICKS = 10_000  # maximum ticks in move segment

MOVE_INSTRUCTION = {'INSTRUCTION': 1, 'TICKS': 7}


def bit_shift(platform):
    '''retrieve bit shif for a give degree

    Determined by running the test of
    movement.py and varying parameter
    '''
    if platform.poldegree == 3:
        bit_shift = 40
    elif platform.poldegree == 2:
        bit_shift = 25
    else:
        raise Exception("Order not supported")
    return bit_shift


def wordsinscanline(bits):
    bytesinline = 8  # Instruction, direction, ticksperstep
    bytesinline += ceil(bits/8)
    return ceil(bytesinline/WORD_BYTES)


def wordsinmove(platform):
    '''calcuates number of words for a single move instruction'''
    bytesingcode = (sum(MOVE_INSTRUCTION.values())
                    + platform.motors*platform.poldegree*WORD_BYTES)
    bytesingcode += bytesingcode % WORD_BYTES
    return ceil(bytesingcode/WORD_BYTES)


def getmovedct(platform):
    dct = MOVE_INSTRUCTION
    for i in range(platform.motors):
        for j in range(platform.poldegree):
            dct.update({f'C{i}{j}': 8})
