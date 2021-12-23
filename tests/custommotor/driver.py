""" BLDC board provided on seperate board see 
    https://hackaday.io/project/21933-prism-laser-scanner/log/201104-prism-motor-bugeja-style
"""
import unittest

from sys import platform
from luna.gateware.utils.cdc import synchronize
from luna.gateware.test import LunaGatewareTestCase, sync_test_case
from nmigen import Elaboratable, Module, Signal
from platforms import TestPlatform


class Driver(Elaboratable):
    ''' Drives 3 pole BLDC motor

        frequency in Hz, rotation frequency of BLDC motor
        state of the motor, wether motor is turned on
    '''
    def __init__(self, platform, frequency=3, top=False):
        """
        platform  -- pass test platform
        frequency -- motor frequency in Hz
        top       -- True if top module
        """
        self.platform = platform
        self.frequency = frequency
        self.on = Signal()
        self.sensor = Signal()
        self.top = top

    def elaborate(self, platform):
        m = Module()
        if platform and self.top:
            board_spi = platform.request("debug_spi")
            # connect to signal out
            m.d.comb += [self.sensor.eq(board_spi.sdo),
                         self.on.eq(board_spi.sdi)]
            bldc = platform.request("bldc")
        else:
            platform = self.platform
            bldc = platform.bldc
        # coil can have 3 states; north, south or not active
        maxcnt = int(platform.laser_var['CRYSTAL_HZ']/self.frequency)
        timer = Signal(maxcnt.bit_length()+1)
        # connect sensor to SPI sdo
        m.d.comb += self.sensor.eq(bldc.sensor)
        # motor has 3 states 
        state = Signal(range(3))
        with m.FSM(reset='INIT', name='algo'):
            with m.State('INIT'):
                with m.If(self.on):
                    m.next = 'ROTATION'
            with m.State('ROTATION'):
                with m.If(timer == maxcnt):
                    m.d.sync += timer.eq(0)
                    with m.If(state==5):
                        m.d.sync += state.eq(0)
                    with m.Else():
                        m.d.sync += state.eq(state+1)
                with m.Else():
                    m.d.sync += timer.eq(timer+1) 
                with m.If(~self.on):
                    m.next = 'INIT'

        # coded by hand, for quick test
        with m.If(state == 0):
            m.d.comb += [bldc.uL.eq(1),
                         bldc.uH.eq(0),
                         bldc.vL.eq(0),
                         bldc.vH.eq(0),
                         bldc.wL.eq(0),
                         bldc.wH.eq(0)]
        with m.Elif(state==1):
            m.d.comb += [bldc.uL.eq(0),
                         bldc.uH.eq(0),
                         bldc.vL.eq(1),
                         bldc.vH.eq(0),
                         bldc.wL.eq(0),
                         bldc.wH.eq(0)]
        with m.Elif(state==2):
            m.d.comb += [bldc.uL.eq(0),
                         bldc.uH.eq(0),
                         bldc.vL.eq(0),
                         bldc.vH.eq(0),
                         bldc.wL.eq(1),
                         bldc.wH.eq(0)]
        with m.Elif(state==3):
            m.d.comb += [bldc.uL.eq(0),
                         bldc.uH.eq(1),
                         bldc.vL.eq(0),
                         bldc.vH.eq(0),
                         bldc.wL.eq(0),
                         bldc.wH.eq(0)]
        with m.Elif(state==4):
             m.d.comb += [bldc.uL.eq(0),
                          bldc.uH.eq(0),
                          bldc.vL.eq(0),
                          bldc.vH.eq(1),
                          bldc.wL.eq(0),
                          bldc.wH.eq(0)]
        with m.Else():
             m.d.comb += [bldc.uL.eq(0),
                          bldc.uH.eq(0),
                          bldc.vL.eq(0),
                          bldc.vH.eq(0),
                          bldc.wL.eq(0),
                          bldc.wH.eq(1)]
        return m

class TestBLDC(LunaGatewareTestCase):
    platform = TestPlatform()
    FRAGMENT_UNDER_TEST = Driver
    FRAGMENT_ARGUMENTS = {'frequency': 3,
                          'platform': platform}

    @sync_test_case
    def test_sensor(self):
        '''counts steps in accounting for direction'''
        dut = self.dut
        yield
        self.assertEqual((yield dut.sensor), 0)
        (yield dut.platform.bldc.sensor.eq(1))
        yield
        self.assertEqual((yield dut.sensor), 1)
    
if __name__ == "__main__":
    unittest.main()