from struct import unpack
from time import sleep
from copy import deepcopy

import spidev
from smbus2 import SMBus
import numpy as np
from gpiozero import LED

from FPGAG.constants import (INSTRUCTIONS, COMMANDS, FREQ, STATE, BIT_SHIFT,
                             MOVE_TICKS, WORD_BYTES, COMMAND_BYTES)
from FPGAG.platforms import Firestarter
from FPGAG.core import Dispatcher
import steppers


class Host:
    'Class for sending instructions to core'

    def __init__(self):
        self.platform = Firestarter()
        # IC bus used to set power laser
        self.bus = SMBus(self.platform.ic_dev_nr)
        # SPI to sent data to scanner
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.mode = 1
        self.spi.max_speed_hz = round(1E6)
        self._position = np.array([0]*self.platform.motors)
        # TODO: this line is key and not well understood
        #       chip select on or of does not affect results
        #       "owning" the pin and transfering ownership from spi is
        #       important
        #       at the moment there is one SPI device so chip select is
        #       not that importatn
        self.chip_select = LED(8)
        self.init_steppers()
        self.enable = LED(self.platform.enable_pin)

    def init_steppers(self):
        '''configure steppers via SPI using teemuatflut CPP library'''
        self.motors = [steppers.TMC2130(link_index=i)
                       for i in range(1, 1+self.platform.motors)]
        steppers.bcm2835_init()
        for motor in self.motors:
            motor.begin()
            motor.toff(5)
            # ideally should be 0
            # on working equipment it is always 2
            assert motor.test_connection() == 2
            motor.rms_current(600)
            motor.microsteps(16)
            motor.en_pwm_mode(True)
        # close bcm2835
        steppers.bcm2835_close()

    def build(self, do_program=True, verbose=True):
        self.platform = Firestarter()
        self.platform.build(Dispatcher(Firestarter()),
                            do_program=do_program, verbose=verbose)

    def reset(self):
        'reset the chip by raising and lowering the reset pin'
        reset_pin = LED(self.platform.reset_pin)
        reset_pin.off()
        sleep(1)
        reset_pin.on()
        sleep(1)

    def _read_state(self):
        '''reads the state and returns bytearray'''
        read_data = (self.send_command([COMMANDS.READ] +
                                       WORD_BYTES*[0]))
        return read_data

    @property
    def position(self):
        '''retrieves and updates position'''
        for i in range(self.platform.motors):
            read_data = (self.send_command([COMMANDS.POSITION] +
                         WORD_BYTES*[0]))
            self._position[i] = unpack('!q', read_data[1:])[0]
        # step --> mm
        self._position = (self._position /
                          np.array(list(self.platform.stepspermm.values())))

        return self._position

    @property
    def pinstate(self):
        '''retrieves pin state as dictionary'''
        data = self._read_state()
        bits = "{:08b}".format(data[-2])
        dct = {'x': int(bits[0]),
               'y': int(bits[1]),
               'z': int(bits[2])}
        return dct

    @property
    def dispatcherror(self):
        '''retrieves dispatch error status of FPGA via SPI'''
        data = self._read_state()
        bits = "{:08b}".format(data[-1])
        return int(bits[STATE.DISPATCHERROR])

    @property
    def enable_steppers(self):
        '''get status enables pin motors

        Execution might still be disabled on the FPGA
        '''
        enable = LED(self.platform.enable_pin)
        return enable.value

    @enable_steppers.setter
    def enable_steppers(self, val):
        '''disable stepper motors

        sets enable pin on raspberry pi platform
        send enable or disable command to FPGA core

        val -- boolean, True enables steppers
        '''
        assert type(val) == bool
        if val:
            self.enable.off()
            self.spi_exchange_data([COMMANDS.START]+WORD_BYTES*[0])
        else:
            self.enable.on()
            self.spi_exchange_data([COMMANDS.STOP]+WORD_BYTES*[0])

    @property
    def execution(self):
        '''determine wether code in SRAM is dispatched

        The dispatcher on the FPGA can be on or off
        '''
        data = (yield from self._read_state())
        bits = "{:08b}".format(data[-1])
        return int(bits[STATE.PARSING])

    def _executionsetter(self, val):
        'not able to call execution with yield from'
        assert type(val) == bool
        command = []
        if val:
            command.append(COMMANDS.START)
        else:
            command.append(COMMANDS.STOP)
        self.send_command(command+WORD_BYTES*[0])

    @execution.setter
    def execution(self, val):
        '''set dispatcher on or of

        val -- True, dispatcher is enabled
        '''
        self._executionsetter(val)

    def home_axes(self, axes, speed, pos=-200):
        '''home given axes

        e.g. axes = [1,0,1] will home x and z
        axes  -- list with axes numbers to home
        speed -- speed in mm/s used to home
        pos   -- position to home to
        '''
        assert len(axes) == self.platform.motors
        dist = np.array(axes)*np.array([pos]*self.platform.motors)
        self.gotopoint(dist, speed)

    def steps_to_count(self, steps):
        '''compute count for a given number of steps

        steps  -- motor moves in small steps

        Shift is needed as two ticks per step are required
        You need to count slightly over the threshold. That is why
        +1 is added.
        '''
        count = (steps << (1+BIT_SHIFT))+(1 << (BIT_SHIFT-1))
        return count

    def gotopoint(self, position, speed, absolute=True):
        '''move steppers to point with constant speed

        postion      -- list with coordinate or distance in mm
        absolute     -- absolute position otherwise coordinate is distance
        speed        -- speed in mm/s
        '''
        assert len(position) == self.platform.motors
        if speed is not None:
            assert len(speed) == self.platform.motors
        else:
            speed = [10]*self.platform.motors
        if absolute:
            dist = np.array(position) - self._position
        else:
            dist = np.array(position)
        speed = np.array(speed)
        # NOTE: ticks is equal for all axes
        t = np.array([np.absolute((dist/speed))[0]])
        ticks = (t*FREQ).round().astype(int)
        steps_per_mm = np.array(list(self.platform.stepspermm.values()))
        # mm -> steps
        dist_steps = dist * steps_per_mm

        def get_ticks(x):
            if x >= MOVE_TICKS:
                return MOVE_TICKS
            else:
                return x
        get_ticks_v = np.vectorize(get_ticks)

        ticks_total = np.copy(ticks)
        steps_total = np.zeros_like(dist_steps, dtype='int64')
        self._executionsetter(True)
        hit_home = np.zeros_like(dist_steps)
        while ticks.sum() > 0:
            ticks_move = get_ticks_v(ticks)
            # steps -> count
            steps_move = dist_steps*(ticks_move/ticks_total)
            steps_move = steps_move.round().astype('int64')
            steps_total += steps_move
            cnts = self.steps_to_count(steps_move)
            a = (cnts/ticks_move).round().astype('int64')
            ticks -= MOVE_TICKS
            ticks[ticks < 0] = 0
            hit_home = self.send_move(ticks_move.tolist(),
                                      a.tolist(),
                                      [0]*self.platform.motors,
                                      [0]*self.platform.motors)
            dist_steps = dist_steps*hit_home
            if dist_steps.sum() == 0:
                break
        dist_mm = steps_total / steps_per_mm
        self._position = (self._position + dist_mm)*hit_home

    def memfull(self, data=None):
        '''check if memory is full

        data -- data received from peripheral
        '''
        if data is None:
            data = self._read_state()
        bits = "{:08b}".format(data[-1])
        return int(bits[STATE.FULL])

    def send_command(self, data, format='!Q'):
        assert len(data) == WORD_BYTES+COMMAND_BYTES
        read_data = self.spi_exchange_data(data)
        return read_data

    def send_move(self, ticks, a, b, c, maxtrials=1E5, delay=0.1):
        '''send move instruction with data

        data            -- coefficients for polynomal move
        maxtrials       -- max number of communcation trials

        returns array with status home switches
        Zero implies home switch is hit
        '''
        commands = self.move_commands(ticks, a, b, c)
        trials = 0
        home_bits = np.ones((self.platform.motors))
        command = commands.pop(0)
        while True:
            data_out = self.send_command(command)
            trials += 1
            bits = [int(i) for i in "{:08b}".format(data_out[-1])]
            if int(bits[STATE.DISPATCHERROR]):
                raise Exception("Parsing error")
            bits = [int(i) for i in "{:08b}".format(data_out[-2])]
            home_bits -= np.array(bits[:self.platform.motors])
            if not self.memfull(data_out):
                break
            if trials > maxtrials:
                raise Exception("Too many trials needed")
        for command in commands:
            self.send_command(command)
        return home_bits

    def move_commands(self, ticks, a, b, c):
        '''get list of commands for move instruction with
           [a,b,c] for ax+bx^2+cx^3

           ticks        -- ticks in move
           speed        -- speed in mm/s
           acceleration -- acceleration in mm/s2
           postion      -- list with position in mm
        '''
        assert len(ticks) == 1
        assert len(a) == len(b) == len(c) == self.platform.motors
        write_byte = COMMANDS.WRITE.to_bytes(1, 'big')
        move_byte = INSTRUCTIONS.MOVE.to_bytes(1, 'big')
        commands = []
        commands += [write_byte +
                     ticks[0].to_bytes(7, 'big') + move_byte]
        for motor in range(self.platform.motors):
            commands += [write_byte + a[motor].to_bytes(8, 'big', signed=True)]
            commands += [write_byte + b[motor].to_bytes(8, 'big', signed=True)]
            commands += [write_byte + c[motor].to_bytes(8, 'big', signed=True)]
        return commands

    def spi_exchange_data(self, data):
        '''writes data to peripheral, returns reply'''
        assert len(data) == (COMMAND_BYTES + WORD_BYTES)
        self.chip_select.off()
        # spidev changes values passed to it
        datachanged = deepcopy(data)
        response = bytearray(self.spi.xfer2(datachanged))
        self.chip_select.on()
        return response
