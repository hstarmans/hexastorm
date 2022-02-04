""" BLDC board provided on seperate board see 
    https://hackaday.io/project/21933-prism-laser-scanner/log/201104-prism-motor-bugeja-style
"""
import itertools
import unittest

from sys import platform
from luna.gateware.utils.cdc import synchronize
from luna.gateware.test import LunaGatewareTestCase, sync_test_case
from nmigen import Elaboratable, Module, Signal
from nmigen.build import ResourceError
from platforms import TestPlatform


class Driver(Elaboratable):
    ''' Drives 3 pole BLDC motor

        frequency in Hz, rotation frequency of BLDC motor
        state of the motor, wether motor is turned on
    '''
    def __init__(self, platform, 
                 frequency=6, states=6, 
                 dutycyclefreq=100000,
                 dutycyclestart=0.8,
                 dutycyclelong=0.01,
                 top=False):
        """
        platform  -- pass test platform
        frequency -- motor frequency in Hz
        dutycycle frequency -- frequency of duty cycle modulation in Hz
        states    -- number of states, typically 6
        dutycycle -- fraction active, number between 0 and 1
        top       -- True if top module
        """
        self.platform = platform
        self.frequency = frequency
        self.dutycyclefleq = dutycyclefreq
        self.states = states
        self.on = Signal()
        self.dutycyclestart = dutycyclestart
        self.dutycyclelong = dutycyclelong
        self.top = top

    def elaborate(self, platform):
        m = Module()

        def get_all_resources(name):
            resources = []
            for number in itertools.count():
                try:
                    resources.append(platform.request(name, number))
                except ResourceError:
                    break
            return resources

        if platform and self.top:
            board_spi = platform.request("debug_spi")
            leds     = [res.o for res in get_all_resources("led")]
            bldc = platform.request("bldc")
            # connect to signal out
            # ALS ie 0 is lees je hoog, als ie laag is lees je hoog
            m.d.comb  += [leds[0].eq(bldc.sensor),
                          board_spi.sdo.eq(bldc.sensor),
                          self.on.eq(board_spi.sdi)]
        else:
            platform = self.platform
            bldc = platform.bldc
        
        maxcnt = int(platform.laser_var['CRYSTAL_HZ']/(self.frequency*self.states))
        maxrotations = 30*self.states*self.frequency
        timer = Signal(maxcnt.bit_length()+1)
        rotations = Signal(range(maxrotations))
        
        state = Signal(range(self.states))
        with m.FSM(reset='INIT', name='algo'):
            with m.State('INIT'):
                m.d.sync += rotations.eq(0)
                # with m.If(self.on):
                m.next = 'ROTATION'
            with m.State('ROTATION'):
                # state
                with m.If(timer == maxcnt):
                    m.d.sync += timer.eq(0)
                    # m.d.sync += state.eq(0)
                    with m.If(state==self.states-1):
                        m.d.sync += state.eq(0)
                    with m.Else():
                        m.d.sync += state.eq(state+1)
                with m.Else():
                    m.d.sync += timer.eq(timer+1) 
                # duty cycle
                with m.If(rotations == maxrotations):
                    m.d.sync += rotations.eq(maxrotations)
                with m.Else():
                    m.d.sync += rotations.eq(rotations+1)
                # with m.If(~self.on):
                #     m.next = 'INIT'
        
        thresh = Signal.like(timer)

        # with m.If(rotations<(10*self.states*self.frequency)):
        m.d.comb += thresh.eq(int(maxcnt*self.dutycyclestart))
        # with m.If(rotations<(10*self.states*self.frequency)):
        #     m.d.comb += thresh.eq(int(maxcnt*self.dutycyclestart))
        # with m.Else():
        #     m.d.comb += thresh.eq(int(maxcnt*self.dutycyclelong))

        # six states and one off state
        with m.If(timer>thresh):
            m.d.comb += [bldc.uL.eq(0),
                         bldc.uH.eq(0),
                         bldc.vL.eq(0),
                         bldc.vH.eq(0),
                         bldc.wL.eq(0),
                         bldc.wH.eq(0)]
        with m.Elif(state == 0):
            m.d.comb += [bldc.uL.eq(0),
                         bldc.uH.eq(0),
                         bldc.vL.eq(0),
                         bldc.vH.eq(1),
                         bldc.wL.eq(1),
                         bldc.wH.eq(0)]
        with m.Elif(state==1):
            m.d.comb += [bldc.uL.eq(1),
                         bldc.uH.eq(0),
                         bldc.vL.eq(0),
                         bldc.vH.eq(1),
                         bldc.wL.eq(0),
                         bldc.wH.eq(0)]
        with m.Elif(state==2):
            m.d.comb += [bldc.uL.eq(1),
                         bldc.uH.eq(0),
                         bldc.vL.eq(0),
                         bldc.vH.eq(0),
                         bldc.wL.eq(0),
                         bldc.wH.eq(1)]
        with m.Elif(state==3):
            m.d.comb += [bldc.uL.eq(0),
                         bldc.uH.eq(0),
                         bldc.vL.eq(1),
                         bldc.vH.eq(0),
                         bldc.wL.eq(0),
                         bldc.wH.eq(1)]
        with m.Elif(state==4):
            m.d.comb += [bldc.uL.eq(0),
                         bldc.uH.eq(1),
                         bldc.vL.eq(1),
                         bldc.vH.eq(0),
                         bldc.wL.eq(0),
                         bldc.wH.eq(0)]
        with m.Elif(state==5):
            m.d.comb += [bldc.uL.eq(0),
                         bldc.uH.eq(1),
                         bldc.vL.eq(0),
                         bldc.vH.eq(0),
                         bldc.wL.eq(1),
                         bldc.wH.eq(0)]
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