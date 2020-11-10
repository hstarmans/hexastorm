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
        cls.STABLE_TIME = round(Scanhead.VARIABLES['SPINUP_TIME']+Scanhead.VARIABLES['STABLE_TIME']+2)

    def test_stable(self):
        'test if laser can stabilize itself and gives memread error if no data is written'
        self.sh.start()
        sleep(self.STABLE_TIME)
        self.stateEqual(state = Scanhead.STATES.START, errors=[Scanhead.ERRORS.MEMREAD])
        self.sh.reset()
    
    def test_photodiode(self):
        'test if photodiode is triggered when laser is turned on and prism spins'
        self.assertFalse(self.sh.test_photodiode())

    def test_stopscanline(self):
        'test if head transitions to stop if a stopline is sent'
        self.sh.writeline([])
        self.sh.start()
        sleep(self.STABLE_TIME)
        self.stateEqual(state = Scanhead.STATES.STOP)
        self.sh.reset()
    
    def stateEqual(self, state, errors=[]):
        val = self.sh.spi.xfer([Scanhead.COMMANDS.STATUS])
        val1 =  self.sh.state(state = state, errors = errors)
        try:
            super().assertEqual(val, val1)
        except AssertionError as e:
            print(e)
            print(self.sh.status(byte=val[0]))
            print('not equal to')
            print(self.sh.status(byte=val1[0]))
            raise

    def test_scanlinerepeated(self):
        '''test scanline with write in single line mode
        
        In this mode the line is always repeated.
        '''
        self.sh.single_line = True
        self.sh.flash(recompile=True, removebuild=True)
        self.sh.writeline([0]*self.sh.sh.BITSINSCANLINE)
        self.sh.start()
        sleep(self.STABLE_TIME)
        self.stateEqual(state = Scanhead.STATES.START)
        # write some lines
        self.sh.writeline([1]*self.sh.sh.BITSINSCANLINE)
        sleep(1)
        self.sh.writeline([0]*self.sh.sh.BITSINSCANLINE)
        sleep(1)
        # write stopline
        self.sh.writeline([])
        sleep(self.STABLE_TIME)
        self.stateEqual(state = Scanhead.STATES.STOP)
        self.sh.single_line = False
        self.sh.flash(recompile=True, removebuild=True)

    def test_scanlinerepeatedsinglefacet(self):
        '''test scanline with write in single line and single facet mode
        
        Scanline is always repeated and only a single facet is used
        Note; real test would require camera, this is added later
        '''
        self.sh.single_line = True
        self.sh.single_facet = True
        self.sh.flash(recompile=True, removebuild=True)
        self.sh.writeline([0]*self.sh.sh.BITSINSCANLINE)
        self.sh.start()
        sleep(self.STABLE_TIME)
        self.sh.writeline([])
        sleep(self.STABLE_TIME)
        self.stateEqual(state = Scanhead.STATES.STOP)
        self.sh.single_line = False
        self.sh.single_facet = False
        self.sh.flash(recompile=True, removebuild=True)

    def test_scanlineringbuffer(self):
        '''test scanline with write using ring buffer
        '''
        self.sh.writeline([0]*self.sh.sh.BITSINSCANLINE)
        self.sh.start()
        sleep(self.STABLE_TIME)
        self.stateEqual(state = Scanhead.STATES.START, errors=[Scanhead.ERRORS.MEMREAD])
        for line in range(100):
            print(f"Writing line number {line}")
            if line%2 == 0:
                res = self.sh.writeline([1]*self.sh.sh.BITSINSCANLINE)
            else:
                res = self.sh.writeline([0]*self.sh.sh.BITSINSCANLINE)
        self.sh.writeline([])
        sleep(self.STABLE_TIME)
        self.stateEqual(state = Scanhead.STATES.STOP, errors=[Scanhead.ERRORS.MEMREAD, Scanhead.ERRORS.INVALID])

    def test_memory(self):
        'test if memory full is raised when writing to memory'
        for _, item in self.sh.get_state().items(): self.assertFalse(item)
        for _ in range(Scanhead.MEMDEPTH//Scanhead.CHUNKSIZE):
            self.assertEqual(self.sh.spi.xfer([Scanhead.COMMANDS.WRITE_L]), self.sh.state(state=Scanhead.STATES.STOP))
            for _ in range(Scanhead.CHUNKSIZE): self.stateEqual(state=Scanhead.STATES.STOP)
        # quick check if you reached end of memory via invalid command
        self.assertEqual(self.sh.spi.xfer([255]), self.sh.state(errors = [Scanhead.ERRORS.MEMFULL], state = Scanhead.STATES.STOP))
        # the above command is invalid and should be captured
        self.stateEqual(errors=[Scanhead.ERRORS.INVALID, Scanhead.ERRORS.MEMFULL], state=Scanhead.STATES.STOP)
        self.sh.reset()
        self.stateEqual(state=Scanhead.STATES.STOP)

if __name__ == '__main__':
    unittest.main()