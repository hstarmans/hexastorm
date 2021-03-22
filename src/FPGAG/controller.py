from struct import unpack

import numpy as np
from gpiozero import LED

from FPGAG.constants import (INSTRUCTIONS, COMMANDS, FREQ, STATE, BIT_SHIFT,
                             MOVE_TICKS, WORD_BYTES, COMMAND_BYTES)
from FPGAG.platforms import Firestarter


class Host:
    'Class for sending instructions to core'

    def __init__(self, platform=None):
        if platform is None:
            self.platform = Firestarter()
        else:
            self.platform = platform
        self._position = np.array([0]*platform.motors)

    def _read_state(self):
        '''reads the state and returns bits'''
        read_data = (yield from self.send_command([COMMANDS.READ] +
                                                  WORD_BYTES*[0]))
        return "{:08b}".format(read_data)

    @property
    def position(self):
        '''retrieves and updates position'''
        for i in range(self.platform.motors):
            read_data = (yield from self.send_command([COMMANDS.POSITION] +
                                                      WORD_BYTES*[0],
                                                      format='!q'))
            self._position[i] = read_data
        # step --> mm
        self._position = (self._position /
                          np.array(list(self.platform.stepspermm.values())))

        return self._position

    @property
    def pinstate(self):
        '''retrieves pin state as dictionary'''
        bits = (yield from self._read_state())
        dct = {'x': int(bits[0]),
               'y': int(bits[1]),
               'z': int(bits[2])}
        return dct

    @property
    def dispatcherror(self):
        '''retrieves dispatch error status of FPGA via SPI'''
        bits = (yield from self._read_state())
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
        enable = LED(self.platform.enable_pin)
        if val:
            enable.on()
            yield from self.spi_exchange_data([COMMANDS.ENABLE]+3*[0])
        else:
            enable.off()
            yield from self.spi_exchange_data([COMMANDS.DISABLE]+3*[0])

    @property
    def execution(self):
        '''determine wether code in SRAM is dispatched

        The dispatcher on the FPGA can be on or off
        '''
        bits = (yield from self._read_state())
        return int(bits[STATE.PARSING])

    def _executionsetter(self, val):
        'not able to call execution with yield from'
        assert type(val) == bool
        command = []
        if val:
            command.append(COMMANDS.START)
        else:
            command.append(COMMANDS.STOP)
        yield from self.send_command(command+WORD_BYTES*[0])

    @execution.setter
    def execution(self, val):
        '''set dispatcher on or of

        val -- True, dispatcher is enabled
        '''
        self._executionsetter(val)

    def home_axes(self, axes, speed, pos=-200):
        '''home given axes

        axes  -- list with axes numbers to home
        speed -- speed in mm/s used to home
        pos   -- position to home to
        '''
        assert len(axes) == self.platform.motors
        dist = np.array(axes)*np.array([pos]*self.platform.motors)
        yield from self.gotopoint(dist, speed)

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
        t = np.absolute((dist/speed))
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
        yield from self._executionsetter(True)
        hit_home = np.zeros_like(dist_steps)
        while ticks.sum() > 0:
            ticks_move = get_ticks_v(ticks)
            # steps -> count
            steps_move = dist_steps*(ticks_move/ticks_total)
            steps_move = steps_move.round().astype('int64')
            steps_total += steps_move
            cnts = (steps_move << (1+BIT_SHIFT))+(1 << (BIT_SHIFT-1))
            a = (cnts/ticks_move).round().astype('int64')
            ticks -= MOVE_TICKS
            ticks[ticks < 0] = 0
            hit_home = (yield from self.send_move(ticks_move.tolist(),
                                                  a.tolist(),
                                                  [0]*self.platform.motors,
                                                  [0]*self.platform.motors))
            dist_steps = dist_steps*hit_home
            if dist_steps.sum() == 0:
                break
        dist_mm = steps_total / steps_per_mm
        self._position = (self._position + dist_mm)*hit_home

    def memfull(self, data):
        '''check if memory is full

        data -- data received from peripheral
        '''
        bits = "{:08b}".format(data & 0xff)
        return int(bits[STATE.FULL])

    def send_command(self, data, format='!Q'):
        assert len(data) == WORD_BYTES+COMMAND_BYTES
        read_data = yield from self.spi_exchange_data(data)
        return unpack(format, read_data[1:])[0]

    def send_move(self, ticks, a, b, c, maxtrials=100):
        '''send move instruction with data

        data            -- coefficients for polynomal move
        maxtrials       -- max number of communcation trials

        returns array with status home switches
        Zero implies home switch is hit
        '''
        commands = self.move_commands(ticks, a, b, c)
        trials = 0
        data_out = 255
        home_bits = np.ones([1]*self.platform.motors)
        while self.memfull(data_out):
            data_out = (yield from self.send_command(commands.pop(0)))
            trials += 1
            bits = [int(i) for i in "{:08b}".format(data_out >> 8)]
            home_bits -= np.array(bits[:self.platform.motors])
            if trials > maxtrials:
                raise Exception("Too many trials needed")
        for command in commands:
            yield from self.send_command(command)
        return home_bits

    def move_commands(self, ticks, a, b, c):
        '''get list of commands for move instruction with
           [a,b,c] for ax+bx^2+cx^3

           ticks        -- ticks in move
           speed        -- speed in mm/s
           acceleration -- acceleration in mm/s2
           postion      -- list with position in mm
        '''
        assert len(ticks) == len(a) == len(b) == len(c) == self.platform.motors
        write_byte = COMMANDS.WRITE.to_bytes(1, 'big')
        move_byte = INSTRUCTIONS.MOVE.to_bytes(1, 'big')
        commands = []
        for motor in range(self.platform.motors):
            commands += [write_byte +
                         ticks[motor].to_bytes(7, 'big') + move_byte]
            commands += [write_byte + a[motor].to_bytes(8, 'big', signed=True)]
            commands += [write_byte + b[motor].to_bytes(8, 'big', signed=True)]
            commands += [write_byte + c[motor].to_bytes(8, 'big', signed=True)]
        return commands

    def spi_exchange_data(data):
        '''writes data to peripheral, returns reply'''
        pass
