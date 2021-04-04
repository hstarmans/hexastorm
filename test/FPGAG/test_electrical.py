import unittest
from time import sleep
from math import floor

from FPGAG.controller import Host
from FPGAG.platforms import Firestarter
from FPGAG.constants import (WORD_BYTES, COMMANDS, MOVE_TICKS,
                             BIT_SHIFT)


class Tests(unittest.TestCase):
    '''Test on a real scanhead with a FPGA'''

    @classmethod
    def setUpClass(cls, flash=True):
        cls.host = Host()
        if flash:
            cls.host.build()
        else:
            print("Resetting the machine")
            cls.host.reset()

    def steps_compute(self, steps):
        '''compute count for a given number of steps

        steps  -- motor moves in small steps

        Shift is needed as two ticks per step are required
        You need to count slightly over the threshold. That is why
        +1 is added.
        '''
        count = (steps << (1+BIT_SHIFT))+(1 << (BIT_SHIFT-1))
        return count

    def readpin(self):
        '''test if you can detect triggers of the limit switches

        This is typically executed manually by placing a sheet of paper
        in the cavity of the optical switch.
        '''
        while True:
            print(self.host.pinstate)
            sleep(1)

    def motorenable(self):
        '''test if motors are enabled and execution is enabled/disabled
           via communication with FPGA'''
        self.host.enable_steppers = True
        self.assertEqual(self.host.execution, True)
        print('check manually if axes are blocked and hard to move')
        input()
        self.host.enable_steppers = False
        self.assertEqual(self.host.execution, False)

    def multiplemove(self):
        '''test if motors move'''
        self.assertEqual(self.host.dispatcherror, False)
        # with false check if this results in block
        # you can do a blink test to verify move
        self.host.enable_steppers = True
        motors = Firestarter.motors
        steps = 10
        print('all axes should move')
        input()
        self.host.gotopoint(position=[1, 1, 1],
                            speed=[1]*3,
                            absolute=False)
        print('retrieving position, should not be zero')
        print(self.host.position)
        input()
        self.host.enable_steppers = False

    def test_memfill(self):
        '''test if memory can be filled and emptied

        Yosys does not raise error for memories which cannot be synthesized
        You fill the entire memory and see if it is properly emptied
        '''
        self.assertEqual(self.host.dispatcherror, False)
        # with false check if this results in block
        # you can do a blink test to verify move
        motors = Firestarter.motors
        steps = 10
        for _ in range(self.platforms.memdepth):
            a = round(self.steps_compute(steps)/MOVE_TICKS)
            self.host.send_move([10000],
                                [a]*motors,
                                [0]*motors,
                                [0]*motors)
            sleep(1)
        self.host.enable_steppers = True
        sleep(4)
        print(self.host.position)
        # print('all axes should move')
        # input()
        # self.host.gotopoint(position=[1, 1, 1],
        #                     speed=[1]*3,
        #                     absolute=False)
        print('position should not be zero which includes error')
        self.host.enable_steppers = False
        self.assertEqual(self.host.dispatcherror, False)

    def test_invalidinstruction(self):
        '''write invalid instruction and verify it passes dispatcher'''
        self.host._executionsetter(True)
        command = [COMMANDS.WRITE] + [0]*WORD_BYTES
        for _ in range(self.host.platform.wordsinmove):
            self.host.send_command(command)
        sleep(3)
        self.host._executionsetter(False)
        # TODO: strange that memory is full!???
        self.assertEqual(self.host.memfull(), True)
        self.assertEqual(self.host.dispatcherror, True)

    def test_memfull(self):
        '''write move instruction until static memory is full'''
        limit = floor(Firestarter.memdepth /
                      self.host.platform.wordsinmove)
        motors = Firestarter.motors
        for _ in range(limit):
            self.host.send_move([1000],
                                [1]*motors,
                                [2]*motors,
                                [3]*motors)
        self.assertEqual(self.host.memfull(), True)


if __name__ == "__main__":
    unittest.main()
