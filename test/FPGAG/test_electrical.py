import unittest
from FPGAG.controller import Host


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

    def test_memfull(self):
        'write move instruction until memory is full'
        for _ in range(1000):
            for _ in self.host.send_move([1000]*3,
                                         [1]*3,
                                         [2]*3,
                                         [3]*3):
                pass
        print('This should break')


if __name__ == "__main__":
    unittest.main()
