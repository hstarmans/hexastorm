import unittest
from time import sleep
from math import floor

import numpy as np
from numpy.testing import assert_array_equal
from FPGAG.controller import Host
from FPGAG.platforms import Firestarter
from FPGAG.constants import (WORD_BYTES, COMMANDS, MOVE_TICKS)


class Tests(unittest.TestCase):
    '''Test on a real scanhead with a FPGA'''

    @classmethod
    def setUpClass(cls, flash=False):
        cls.host = Host()
        if flash:
            cls.host.build()
        else:
            print("Resetting the machine")
            cls.host.reset()
            
    def _executor(func):
        '''executes generator until stop iteration
        
        Nmigen uses generator syntax and this leaks into our
        python code. As a result, it is required to iterate.
        '''
        def inner(self):
            iteration = 0
            for _ in func(self):
                pass
        return inner    
    
    @_executor
    def readpin(self):
        '''test if you can detect triggers of the limit switches

        This is typically executed manually by placing a sheet of paper
        in the cavity of the optical switch.
        '''
        self.host.enable_steppers = False
        while True:
            print((yield from self.host.pinstate))
            sleep(1)
    
    @_executor
    def motorenable(self):
        '''test if motors are enabled and execution is enabled/disabled
           via communication with FPGA'''
        self.host.enable_steppers = True
        self.assertEqual((yield from self.host.execution), True)
        print('check manually if axes are blocked and hard to move')
        input()
        self.host.enable_steppers = False
        self.assertEqual((yield from self.host.execution), False)
    
    @_executor
    def multiplemove(self):
        '''test if motors move'''
        motors = Firestarter.motors
        position = np.array([10, 10, 0])
        self.assertEqual((yield from self.host.dispatcherror), False)
        self.host.enable_steppers = True
        yield from self.host.gotopoint(position=position,
                                       speed=[1]*motors,
                                       absolute=False)
        # NOTE: is the sleep really needed?
        print('sleep to complete')
        sleep(1)
        assert_array_equal((yield from self.host.position),
                           position)
        self.host.enable_steppers = False
    
    @_executor
    def test_memfull(self):
        '''test if memory can be filled and emptied

        Yosys does not raise error for memories which cannot be synthesized
        You fill the entire memory and see if it is properly emptied
        by ensuring that the position changes as excepted.
        '''
        self.assertEqual((yield from self.host.dispatcherror), False)
        self.host.enable_steppers = False
        # with false check if this results in block
        # you can do a blink test to verify move
        motors = Firestarter.motors
        host = self.host
        mm = np.array([1]*motors)
        steps = mm * np.array(list(host.platform.stepspermm.values()))
        limit = floor(host.platform.memdepth /
                      host.platform.wordsinmove)
        for _ in range(limit):
            a = ((host.steps_to_count(steps.astype('int64'))/MOVE_TICKS)
                 .round().astype('int64'))
            yield from host.send_move([MOVE_TICKS],
                                      a.tolist(),
                                      [0]*motors,
                                      [0]*motors)
        self.assertEqual((yield from self.host.memfull()), True)
        yield from self.host._executionsetter(True)
        sleep(1)
        self.assertEqual((yield from self.host.memfull()), False)
        assert_array_equal((yield from self.host.position),
                           mm*limit)
        self.host._executionsetter(False)
        self.assertEqual((yield from self.host.dispatcherror), False)
    
    @_executor
    def test_invalidinstruction(self):
        '''write invalid instruction and verify it passes dispatcher'''
        yield from self.host._executionsetter(True)
        command = [COMMANDS.WRITE] + [0]*WORD_BYTES
        for _ in range(self.host.platform.wordsinmove):
            yield from self.host.send_command(command)
        sleep(3)
        yield from self.host._executionsetter(False)
        self.assertEqual((yield from self.host.memfull()), False)
        self.assertEqual((yield from self.host.dispatcherror), True)



if __name__ == "__main__":
    unittest.main()
