import unittest

from migen import *
from litex.soc.cores.spi import SPIMaster

from spi_statemachine import LEDProgram

class TestSPIStateMachine(unittest.TestCase):
    def setUp(self):
        class DUT(Module):
            def __init__(self):
                pads = Record([("clk", 1), ("cs_n", 1), ("mosi", 1), ("miso", 1)])
                self.submodules.master = SPIMaster(pads, data_width=8,
                        sys_clk_freq=100e6, spi_clk_freq=5e6,
                        with_csr=False)
                self.led = Signal()
                self.submodules.ledprogram = LEDProgram(pads, self.led, memdepth=16)
        self.dut = DUT()

    def transaction(self, data_sent, data_received):
        ''' 
        helper function to test transaction from raspberry pi side
        '''
        yield self.dut.master.mosi.eq(data_sent)
        yield self.dut.master.length.eq(8)
        yield self.dut.master.start.eq(1)
        yield
        yield self.dut.master.start.eq(0)
        yield
        while (yield self.dut.master.done) == 0:
            yield
        self.assertEqual((yield self.dut.master.miso), data_received)

    def test_ledstatechange(self):
        def raspberry_side():
            # get the initial status
            yield from self.transaction(LEDProgram.COMMANDS.STATUS, 0)
            # turn on the LED, status should still be zero
            yield from self.transaction(LEDProgram.COMMANDS.START, 0)
            # check wether the led is on
            # error is reported as nothing has been written yetS
            yield from self.transaction(LEDProgram.COMMANDS.STATUS, int('100010',2))
            # turn OFF the led state machine
            yield from self.transaction(LEDProgram.COMMANDS.STOP, int('100010',2))
            # LED state machine should be off and the led off
            yield from self.transaction(LEDProgram.COMMANDS.STATUS, 0)

        def fpga_side():
            timeout = 0
            # LED statemachine should be off on the start
            self.assertEqual((yield self.dut.ledprogram.ledstate), 0)
            # wait till led state changes
            while (yield self.dut.ledprogram.ledstate) == 0:
                timeout += 1
                if timeout>1000:
                    raise Exception("Led doesn't turn on.")
                yield
            timeout = 0
            # LED statemachine should be one now
            # Wether LED is on depends on data
            self.assertEqual((yield self.dut.ledprogram.ledstate), 1)
            # LED should be off
            self.assertEqual((yield self.dut.led), 0)
            # wait till led state changes
            while (yield self.dut.ledprogram.ledstate) == 1:
                timeout += 1
                if timeout>1000:
                    raise Exception("Led doesn't turn off.")
                yield
            # LED statemachine should be off
            self.assertEqual((yield self.dut.ledprogram.ledstate), 0)
        run_simulation(self.dut, [raspberry_side(), fpga_side()])

    def test_writedata(self):
        def raspberry_side():
            # write lines to memory
            for i in range(self.dut.ledprogram.MEMDEPTH+1):
                data_byte = i%256 # bytes can't be larger than 255
                if i%(LEDProgram.CHUNKSIZE)==0:
                    if (i>0)&((i%self.dut.ledprogram.MEMDEPTH)==0):
                        # check if memory is full
                        yield from self.transaction(LEDProgram.COMMANDS.WRITE_L, 1)
                        continue
                    else:
                        yield from self.transaction(LEDProgram.COMMANDS.WRITE_L, 0)
                yield from self.transaction(data_byte, 0)
            # memory is tested in litex
            in_memory = []
            loops = 10
            for i in range(loops):
                value = (yield self.dut.ledprogram.mem[i])
                in_memory.append(value)
            self.assertEqual(list(range(loops)),in_memory)
        run_simulation(self.dut, [raspberry_side()])

    def test_ledstatechangepostwrite(self):
        def raspberry_side():
            # get the initial status
            yield from self.transaction(LEDProgram.COMMANDS.STATUS, 0)
            # write lines to memory
            for i in range(self.dut.ledprogram.MEMDEPTH+1):
                data_byte = i%256 # bytes can't be larger than 255
                if i%(LEDProgram.CHUNKSIZE)==0:
                    if (i>0)&((i%self.dut.ledprogram.MEMDEPTH)==0):
                        # check if memory is full
                        yield from self.transaction(LEDProgram.COMMANDS.WRITE_L, 1)
                        continue
                    else:
                        yield from self.transaction(LEDProgram.COMMANDS.WRITE_L, 0)
                yield from self.transaction(data_byte, 0)
            # turn on the LED, status should be one as memory is full
            yield from self.transaction(LEDProgram.COMMANDS.START, 1)
            # ledstate should change
            timeout = 0
            while (yield self.dut.ledprogram.ledstate) == 0:
                timeout += 1
                if timeout>1000:
                    raise Exception("Led state doesnt go on.")
                yield
            # status should be memory full and led on
            yield from self.transaction(LEDProgram.COMMANDS.STATUS, int('100001',2))
            # led should turn on now
            timeout = 0
            while (yield self.dut.led) == 0:
                timeout += 1
                if timeout>1000:
                    raise Exception("Led doesn't turn on.")
                yield
            # you know LED is on now
            # LED should be on for three ticks
            count = 0
            while (yield self.dut.led) == 1:
                count += 1
                if count>1000:
                    raise Exception("Led doesn't turn on.")
                yield
            self.assertEqual(count, self.dut.ledprogram.MAXPERIOD)
            # you could count until led is zero and then 1 again as check
            # check if you receive read errorS
            while (yield self.dut.ledprogram.error) == 0:
                timeout += 1
                if timeout>1000:
                    raise Exception("Don't receive read error.")
                yield
            # status should be memory empty, led statemachine on and read error
            # you do get an error so written must zero
            yield from self.transaction(LEDProgram.COMMANDS.STATUS, int('100010',2))
        run_simulation(self.dut, [raspberry_side()])

if __name__ == '__main__':
    unittest.main()