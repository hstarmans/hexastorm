import unittest
from time import sleep

from hexastorm.core import Scanhead
from hexastorm.controller import Machine

class Tests(unittest.TestCase):
    ''' Test on a real scanhead with a FPGA'''

    @classmethod
    def setUpClass(cls, flash=True):
        cls.sh = Machine()
        if flash:
            cls.sh.flash(recompile=True, removebuild=True)
        else:
            print("Resetting the machine")
            cls.sh.reset()
        cls.STABLE_TIME = round(Scanhead.VARIABLES['SPINUP_TIME']+Scanhead.VARIABLES['STABLE_TIME']+2)

    def stable(self):
        'test if laser can stabilize itself and gives memread error if no data is written'
        self.sh.start()
        sleep(self.STABLE_TIME)
        self.stateEqual(state = Scanhead.STATES.START, errors=[Scanhead.ERRORS.MEMREAD])
        self.sh.reset()
    
    def photodiode(self):
        'test if photodiode is triggered when laser is turned on and prism spins'
        self.assertFalse(self.sh.test_photodiode())

    def stopscanline(self):
        'test if head transitions to stop if a stopline is sent'
        self.sh.writeline([])
        self.sh.start()
        sleep(self.STABLE_TIME)
        self.stateEqual(state = Scanhead.STATES.STOP)
        self.sh.reset()
    
    def stateEqual(self, state, errors=[]):
        '''helper function which asserts if machine is in given state
            NOTE: not a test should be moved
        '''
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

    def scanlinerepeated(self):
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

    def scanlinerepeatedsinglefacet(self):
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

    def scanlineringbuffer(self):
        '''test scanline with write using ring buffer
        '''
        #there can be 10 lines in memory
        maximum = 4
        for i in range(maximum):
            print(i)
            self.sh.writeline([0]*self.sh.sh.BITSINSCANLINE)
        self.sh.start()
        sleep(self.STABLE_TIME+2)
        # for line in range(100):
        #     print(f"Writing line number {line}")
        #     if line%2 == 0:
        #         self.sh.writeline([1]*self.sh.sh.BITSINSCANLINE)
        #     else:
        #         self.sh.writeline([0]*self.sh.sh.BITSINSCANLINE)
            #TODO: if you don't sleep between lines --> it will crash
            #      system needs empty memory, your memory full doesn't propagate fast enough
            # sleep(0.1)
        # self.sh.writeline([])
        sleep(self.STABLE_TIME)
        self.stateEqual(state = Scanhead.STATES.STOP)

    def memory(self):
        '''test if memory full is raised when writing to memory'''
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