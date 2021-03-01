from collections import namedtuple
import numpy as np

COMMANDS = namedtuple('COMMANDS', ['EMPTY', 'GCODE', 'STATUS', 'START', 'STOP'],
                      defaults=range(5))()
STATE = namedtuple('STATE', ['FULL', 'DISPATCHERROR'], defaults=range(2))()
# one block is 4K there are 32 blocks (officially 20 in HX4K)
# max of 1 block is 16*256 but if you use a larger memwidth you seem to
# use more blocks so this might work
MEMWIDTH = 32
COMMAND_SIZE = 8
WORD_SIZE = 32
WORD_BYTES = round(WORD_SIZE/8)
FREQ = 1 # speed of motor speed update in Mhz
G_CODE = {'COMMAND': 1, 'AUX': 1, 'EMTY': 2}
BEZIER_DEGREE = 2

# TODO: move this to board
VARIABLES = {'CRYSTAL_HZ': 50E6}

def getbytesingcode(motors):
    bytesingcode =(sum(G_CODE.values())
                   + motors*sum(MOTOR_COMMAND.values()))
    bytesingcode += WORD_BYTES-bytesingcode%WORD_BYTES
    return bytesingcode
                          
def getgcodecommanddct(motors):
    dct = G_CODE
    for i in range(motors):
        motor_command = {f'B{i}0': 1}
        for j in range(BEZIER_DEGREE):
            motor_command.update({f'B{i}{j+1}':1)
        dct.update(MOTOR_COMMAND)




# NOTE: following doesnt work due to bug in pylint https://github.com/PyCQA/pylint/issues/3876
# def customnamedtuple(typename, field_names) -> namedtuple:
#    return namedtuple(typename, field_names,
#                      defaults=range(len(field_names)))
