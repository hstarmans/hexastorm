import unittest

from hexastorm.core import Scanhead
from hexastorm.controller import Machine


class TestScanhead(unittest.TestCase):
    ''' Test on scanhead flashed to FPGA'''
    
    def setUp(self):
        self.sh = Machine()
        self.sh.flash(recompile=True, removebuild=True)
    
    def test_photodiode(self):
        assert self.sh.test_photodiode() == False

    def test_memory(self):
        for _ in range(Scanhead.MEMDEPTH//Scanhead.MEMWIDTH):
            assert self.sh.spi.xfer([Scanhead.COMMANDS.WRITE_L]) == self.sh.state(state=Scanhead.STATES.STOP)
            for _ in range(Scanhead.CHUNKSIZE): assert self.sh.spi.xfer([int('11111111', 2)]) == self.sh.state(state=Scanhead.STATES.STOP)
        # quick check if you reached end of chunk via invalid command
        assert self.sh.spi.xfer([255]) == self.sh.state(errors = [Scanhead.ERRORS.MEMFULL], state = Scanhead.STATES.STOP)
        # the above command is invalid and should be captured
        assert self.sh.spi.xfer([Scanhead.COMMANDS.STATUS]) == self.sh.state(errors=[Scanhead.ERRORS.INVALID, Scanhead.ERRORS.MEMFULL],
                                                                             state=Scanhead.STATES.STOP)
        self.sh.reset()
        self.sh.spi.xfer([Scanhead.COMMANDS.STATUS]) == self.sh.state(state=Scanhead.STATES.STOP)


if __name__ == '__main__':
    unittest.main()