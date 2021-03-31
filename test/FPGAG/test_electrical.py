import unittest
from time import sleep

from FPGAG.controller import Host
from FPGAG.platforms import Firestarter
from FPGAG.constants import DEGREE


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

    def readpin(self):
        '''test if you can detect triggers of the limit switches

        This is typically executed manually by placing a sheet of paper
        in the cavity of the optical switch.
        '''
        while True:
            print(self.host.pinstate)
            sleep(1)

    def motorenable(self):
        '''test if motors are enabled'''
        self.host.enable_steppers = True
        print('check manually if axes are blocked and hard to move')
        input()
        self.host.enable_steppers = False

    def test_memfull(self):
        '''write move instruction until static memory is full'''
        limit = round(Firestarter.memdepth/(Firestarter.motors*(DEGREE+1)))
        motors = Firestarter.motors
        for _ in range(limit):
            self.host.send_move([1000]*motors,
                                [1]*motors,
                                [2]*motors,
                                [3]*motors)
        self.assertEqual(self.host.memfull(), True)


if __name__ == "__main__":
    unittest.main()
