import unittest
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

    def test_readpin(self):
        '''test if you can detect triggers of the limit switches'''
        from time import sleep
        while True:
            print(self.host.pinstate)
            sleep(1)

    def test_memfull(self):
        '''write move instruction until memory is full'''
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
