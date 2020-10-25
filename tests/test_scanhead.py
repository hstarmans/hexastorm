import unittest
from time import sleep

from hexastorm.core import Scanhead
from hexastorm.controller import Machine

class TestScanhead(unittest.TestCase):
    ''' Test on scanhead flashed to FPGA'''

    @classmethod
    def setUpClass(cls):
        cls.sh = Machine()
        cls.sh.flash(recompile=True, removebuild=True)

    def test_stable(self):
        'test if laser can stabilize itself and gives memread error if no data is written'
        self.sh.start()
        sleep(round(Scanhead.VARIABLES['SPINUP_TIME']+Scanhead.VARIABLES['STABLE_TIME']+2))
        assert self.sh.spi.xfer([Scanhead.COMMANDS.STATUS]) == self.sh.state(state = Scanhead.STATES.START,
                                                                             errors=[Scanhead.ERRORS.MEMREAD])
        self.sh.reset()
        self.sh.spi.xfer([Scanhead.COMMANDS.STATUS]) == self.sh.state(state=Scanhead.STATES.STOP)
    
    def test_photodiode(self):
        'test if photodiode is triggered when laser is turned on and prism spins'
        assert self.sh.test_photodiode() == False

    def test_memory(self):
        'test if memory full is raised when writing to memory'
        assert self.sh.get_state() == 0
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