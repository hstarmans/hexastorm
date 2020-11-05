import unittest
from time import sleep

from hexastorm.core import Scanhead
from hexastorm.controller import Machine

class TestScanhead(unittest.TestCase):
    ''' Test on scanhead flashed to FPGA'''

    @classmethod
    def setUpClass(cls, flash=True):
        cls.sh = Machine()
        if flash:
            cls.sh.flash(recompile=True, removebuild=True)
        else:
            cls.sh.reset()

    def test_stable(self):
        'test if laser can stabilize itself and gives memread error if no data is written'
        self.sh.start()
        sleep(round(Scanhead.VARIABLES['SPINUP_TIME']+Scanhead.VARIABLES['STABLE_TIME']+2))
        self.assertEqual(self.sh.spi.xfer([Scanhead.COMMANDS.STATUS]), self.sh.state(state = Scanhead.STATES.START,errors=[Scanhead.ERRORS.MEMREAD]))
        self.sh.reset()
    
    def test_photodiode(self):
        'test if photodiode is triggered when laser is turned on and prism spins'
        self.assertFalse(self.sh.test_photodiode())

    def test_stopscanline(self):
        'test if head transitions to stop if a stopline is sent'
        self.sh.writeline([])
        self.sh.start()
        sleep(round(Scanhead.VARIABLES['SPINUP_TIME']+Scanhead.VARIABLES['STABLE_TIME']+2))
        self.assertEqual(self.sh.spi.xfer([Scanhead.COMMANDS.STATUS]), self.sh.state(state = Scanhead.STATES.STOP))
        self.sh.reset()

    def test_memory(self):
        'test if memory full is raised when writing to memory'
        for _, item in self.sh.get_state().items(): self.assertFalse(item)
        for _ in range(Scanhead.MEMDEPTH//Scanhead.CHUNKSIZE):
            self.assertEqual(self.sh.spi.xfer([Scanhead.COMMANDS.WRITE_L]), self.sh.state(state=Scanhead.STATES.STOP))
            for _ in range(Scanhead.CHUNKSIZE): self.assertEqual(self.sh.spi.xfer([int('11111111', 2)]), self.sh.state(state=Scanhead.STATES.STOP))
        # quick check if you reached end of memory via invalid command
        self.assertEqual(self.sh.spi.xfer([255]), self.sh.state(errors = [Scanhead.ERRORS.MEMFULL], state = Scanhead.STATES.STOP))
        # the above command is invalid and should be captured
        self.assertEqual(self.sh.spi.xfer([Scanhead.COMMANDS.STATUS]), self.sh.state(errors=[Scanhead.ERRORS.INVALID,
                                                                                     Scanhead.ERRORS.MEMFULL],
                                                                                    state=Scanhead.STATES.STOP))
        self.sh.reset()
        self.sh.spi.xfer([Scanhead.COMMANDS.STATUS]) == self.sh.state(state=Scanhead.STATES.STOP)

if __name__ == '__main__':
    unittest.main()