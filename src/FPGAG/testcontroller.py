from struct import unpack

import numpy as np
from FPGAG.constants import (INSTRUCTIONS, COMMANDS, FREQ, STATE, BIT_SHIFT,
                             MOVE_TICKS, WORD_BYTES, COMMAND_BYTES)


class TestHost:
    'Test class for sending instructions to core'

    def __init__(self, platform):
        self.platform = platform
        self._position = np.array([0]*self.platform.motors)

    def _read_state(self):
        '''reads the state and returns bytearray'''
        read_data = (yield from self.send_command([COMMANDS.READ] +
                                                  WORD_BYTES*[0]))
        return read_data

    @property
    def position(self):
        '''retrieves and updates position'''
        for i in range(self.platform.motors):
            read_data = (yield from self.send_command([COMMANDS.POSITION] +
                                                      WORD_BYTES*[0]))
            self._position[i] = unpack('!q', read_data[1:])[0]
        # step --> mm
        self._position = (self._position /
                          np.array(list(self.platform.stepspermm.values())))

        return self._position

    @property
    def pinstate(self):
        '''retrieves pin state as dictionary'''
        data = (yield from self._read_state())
        bits = "{:08b}".format(data[-2])
        dct = {'x': int(bits[0]),
               'y': int(bits[1]),
               'z': int(bits[2])}
        return dct

    @property
    def dispatcherror(self):
        '''retrieves dispatch error status of FPGA via SPI'''
        data = (yield from self._read_state())
        bits = "{:08b}".format(data[-1])
        return int(bits[STATE.DISPATCHERROR])

    @property
    def enable_steppers(self):
        '''get status enables pin motors

        Execution might still be disabled on the FPGA
        '''
        pass

    @enable_steppers.setter
    def enable_steppers(self, val):
        '''disable stepper motors

        sets enable pin on raspberry pi platform
        send enable or disable command to FPGA core

        val -- boolean, True enables steppers
        '''
        assert type(val) == bool
        if val:
            yield from self.spi_exchange_data([COMMANDS.ENABLE]+3*[0])
        else:
            yield from self.spi_exchange_data([COMMANDS.DISABLE]+3*[0])

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
        bits = "{:08b}".format(data[-1])
        return int(bits[STATE.FULL])

    def send_command(self, data):
        assert len(data) == WORD_BYTES+COMMAND_BYTES
        read_data = yield from self.spi_exchange_data(data)
        return read_data

    def send_move(self, ticks, a, b, c, maxtrials=100):
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
            data_out = (yield from self.send_command(command))
            trials += 1
            bits = [int(i) for i in "{:08b}".format(data_out[-2])]
            home_bits -= np.array(bits[:self.platform.motors])
            if not self.memfull(data_out):
                break
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
        response = bytearray()
        for byte in data:
            response.append(self.spi.xfer([byte])[0])
        return response
