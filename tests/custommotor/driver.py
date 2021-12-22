""" Sketch of a BLDC driver
"""

from nmigen import Elaboratable, Module, Signal, Array
from hexastorm.platforms import Firestarter


class Driver(Elaboratable):
    ''' Drives 3 pole BLDC motor

        frequency in Hz, rotation frequency of BLDC motor
        state of the motor, wether motor is turned on
    '''
    def __init__(self, frequency=3, on=False):
        self.frequency = frequency
        self.on = Signal()

    def elaborate(self, platform):
        m = Module()
        # coil can have 3 states; north, south or not active
        bldc = platform.bldc
        maxcnt = platform.laser_var['CRYSTAL_HZ']/self.frequency
        timer = Signal(maxcnt.bit_width()+1)
        # motor has 3 states 
        state = Signal(range(3))
        with m.FSM(reset='INIT') as algo:
            with m.State('INIT'):
                with m.If(self.on):
                    m.next = 'ROTATION'
            with m.State('ROTATION'):
                with m.If(timer = maxcnt):
                    m.d.sync += timer.eq(0)
                    with m.If(state=5):
                        m.d.sync += state.eq(0)
                    with m.Else():
                        m.d.sync += state.eq(state+1)
                with m.Else():
                    m.d.sync += timer.eq(timer+1) 
                with m.If(~self.on):
                    m.next = 'INIT'

        # coded by hand, for quick test
        with m.If(state==0):
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
