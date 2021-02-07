from collections import namedtuple

COMMANDS = namedtuple('COMMANDS', ['EMPTY', 'GCODE', 'EXIT', 'ABORT'],
                      defaults=range(4))()
MEMWIDTH = 16
MEMDEPTH = 256
COMMAND_SIZE =8
WORD_SIZE = 32
QUEUE_ELEMENT_BYTES = 30


VARIABLES = {'CRYSTAL_HZ': 50E6}


# NOTE: following doesnt work due to bug in pylint https://github.com/PyCQA/pylint/issues/3876
# def customnamedtuple(typename, field_names) -> namedtuple:
#    return namedtuple(typename, field_names,
#                      defaults=range(len(field_names)))
