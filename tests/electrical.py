import unittest
from time import sleep

from hexastorm.core import Scanhead
from hexastorm.controller import Machine

class Tests(unittest.TestCase):
    '''Test on a real scanhead with a FPGA'''

    @classmethod
    def setUpClass(cls, flash=True):
        cls.sh = Machine()
        if flash:
            cls.sh.flash(recompile=True, removebuild=True)
        else:
            print("Resetting the machine")
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
        'helper function which asserts if machine is in given state'
        val = self.sh.spi.xfer([Scanhead.COMMANDS.STATUS])[0]
        val1 =  self.sh.statetobyte(state = state, errors = errors)
        try:
            super().assertEqual(val, val1)
        except AssertionError:
            self.sh.status(byte=val)  # prints text
            print('not equal to')
            self.sh.status(byte=val1) # prints text
            self.sh.reset()
            raise Exception("states not equal")

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
        'test scanline with write using ring buffer'
        maximum = 4
        for line in range(maximum):
            print(f"Writing line number {line}")
            self.sh.writeline([0]*self.sh.sh.BITSINSCANLINE)
        self.sh.start()
        for line in range(maximum, 1000+maximum):
            print(f"Writing line number {line}")
            if line%2 == 0:
                self.sh.writeline([1]*self.sh.sh.BITSINSCANLINE)
            else:
                self.sh.writeline([0]*self.sh.sh.BITSINSCANLINE)
        self.sh.writeline([])
        sleep(self.STABLE_TIME)
        #TODO: fast writing seems to cause "invalid bytes"
        #      ringbuffer seems to work correctly
        self.stateEqual(state = Scanhead.STATES.STOP)

    def test_memory(self):
        'test if memory full is raised when writing to memory'
        #TODO: work with CHUNKsize!
        for i in range((self.sh.sh.MEMDEPTH-self.sh.sh.bytesinline)//self.sh.sh.CHUNKSIZE):
            self.assertEqual(self.sh.spi.xfer([Scanhead.COMMANDS.WRITE_L])[0], self.sh.statetobyte(state=Scanhead.STATES.STOP))
            for _ in range(self.sh.sh.CHUNKSIZE): self.stateEqual(state=Scanhead.STATES.STOP)
        # quick check if you reached end of memory via invalid command
        self.assertEqual(self.sh.spi.xfer([255])[0], self.sh.statetobyte(errors = [Scanhead.ERRORS.MEMFULL], state = Scanhead.STATES.STOP))
        # the above command is invalid and should be captured
        self.stateEqual(errors=[Scanhead.ERRORS.INVALID, Scanhead.ERRORS.MEMFULL], state=Scanhead.STATES.STOP)
        self.sh.reset()
        self.stateEqual(state=Scanhead.STATES.STOP)

if __name__ == '__main__':
    unittest.main()