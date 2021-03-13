from FPGAG.constants import INSTRUCTIONS, COMMANDS


class Host:
    'Class for sending instructions to core'
    ic_dev_nr = 1
    ic_address = 0x28

    def __init__(self):
        pass

    def home_axes(axes, speed):
        '''home given axes

        axes -- list with axes numbers to home
        speed -- speed in mm/s used to home
        '''
        pass

    def enable_steppers(self):
        '''
        enables stepper motors
        '''
        pass

    def disable_steppers(self):
        '''
        disables stepper motors by setting enable pin to high
        '''
        pass

    def gotopoint(position, absolue=True,
                  speed=0, acceleration=0):
        '''move steppers to point

        speed -- speed in mm/s
        acceleration -- acceleration in mm/s2
        postion -- list with position in mm
        '''
        pass

    def move_data(a, b, c):
        '''get data for move instruction with
           [a,b,c] for ax+bx^2+cx^3

           speed -- speed in mm/s
           acceleration -- acceleration in mm/s2
           postion -- list with position in mm
        '''
        data = [COMMANDS.WRITE, INSTRUCTIONS.MOVE, 0, 0, 0]
        data += [a.to_bytes(4, byteorder='big', signed=True)]
        data += [b.to_bytes(4, byteorder='big', signed=True)]
        data += [c.to_bytes(4, byteorder='big', signed=True)]
        return data

    def spi_exchange_data(data):
        '''writes data to peripheral, returns reply'''
        pass
