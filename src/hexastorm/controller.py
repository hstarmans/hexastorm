try:
    from micropython import const
    upython = True
except ImportError:
    upython = False

from struct import unpack
from time import sleep

if upython:
    from .constants import platform as platformmicro
    from ulab import numpy as np
else:
    from .platforms import Firestarter
    import numpy as np

from .constants import (INSTRUCTIONS, COMMANDS, MOTORFREQ, STATE,
                        MOVE_TICKS, WORD_BYTES, bit_shift,
                        COMMAND_BYTES, params)


def executor(func):
    '''executes generator until stop iteration

    nMigen uses generator syntax and this is used
    as hack to execute functions.
    '''
    def inner(self):
        for _ in func(self):
            pass
    return inner


class Memfull(Exception):
    '''SRAM memory of FPGA, i.e. FIFO, is full

    Exception is raised when the memory is full.
    '''
    pass


class Host:
    '''Class to interact with FPGA
    '''
    def __init__(self, platform=None):
        '''  platform  -- object which has gateware settings
                          only passed to controller if virtual
                          test is executed. Needed in lasers.py
                          as each test here has a slightly
                          different TestPlatform
        '''
        # case raspberry
        if platform is None:
            self.test = False
            from gpiozero import LED
            import spidev
            from smbus2 import SMBus
            self.platform = Firestarter(micropython=False)
            # IC bus used to set power laser
            self.bus = SMBus(self.platform.ic_dev_nr)
            # SPI to sent data to scanner
            self.spi = spidev.SpiDev()
            self.spi.open(*self.platform.spi_dev)
            self.spi.mode = 1
            self.spi.max_speed_hz = round(1E6)
            self.chip_select = LED(self.platform.chip_select)
            # programs TMC2130
            self.init_steppers()
            # stepper motor enable pin
            self.enable = LED(self.platform.enable_pin)
        # case micropython:
        elif upython:
            self.test = False
            import machine
            self.platform = platformmicro(micropython=True)
            # IC bus
            self.bus = machine.I2C(self.platform.ic_dev_nr)
            self.bus.init(machine.I2C.CONTROLLER, adr=self.platform.ic_addr)
            # SPI
            # spi port is hispi
            self.spi = machine.SPI(self.spi_dev, baudrate=round(1E6))
            self.chip_select = machine.Pin(self.platform.chip_select)
            # program TMC2130
            # TODO: add TMC2130 library to micropython
            # self.init_steppers()
            # stepper motor enable pin
            self.enable = machine.Pin(self.platform.enable_pin)
        else:
            self.platform = platform
            self.test = True
        # maximum number of times tried to write to FIFO
        # if memoery is full
        self.maxtrials = 10 if self.test else 1E5
        self.laser_params = params(self.platform)
        self._position = np.array([0]*self.platform.motors, dtype='float64')

    def init_steppers(self):
        '''configure TMC2130 steppers via SPI

        Uses teemuatflut CPP library with custom python wrapper
            https://github.com/hstarmans/TMCStepper
        '''
        import steppers
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
        steppers.bcm2835_close()

    def build(self, do_program=True, verbose=True):
        '''builds the FPGA code using nMigen, Yosys, Nextpnr and icepack

           do_program  -- flashes the FPGA chip using fomu-flash,
                          resets aftwards
           verbose     -- prints output of Yosys, Nextpnr and icepack
        '''
        if upython:
            print("Micropython cannot update binary, using stored one")
        else:
            import hexastorm.core as core
            self.platform = Firestarter()
            self.platform.laser_var = self.laser_params
            self.platform.build(core.Dispatcher(self.platform),
                                do_program=do_program,
                                verbose=verbose)
        if do_program:
            self.reset()

    def reset(self):
        'restart the FPGA by flipping the reset pin'
        if upython:
            import machine
            reset_pin = machine.Pin(self.platform.reset_pin)
        else:
            from gpiozero import LED
            reset_pin = LED(self.platform.reset_pin)
        reset_pin.off()
        sleep(1)
        reset_pin.on()
        sleep(1)
        # a blank needs to be send, Statictest succeeds but
        # testlaser fails in test_electrical.py
        # on HX4K this was not needed
        # is required for the UP5K
        self.spi_exchange_data([0]*(WORD_BYTES+COMMAND_BYTES))

    def get_motordebug(self):
        '''retrieves the motor debug word

           This is used to debug the PI controller and
           set the correct setting for the Hall interpolation
        '''
        command = [COMMANDS.DEBUG] + WORD_BYTES*[0]
        response = (yield from self.send_command(command))[1:]

        clock = int(self.platform.clks[self.platform.hfosc_div]*1E6)
        mode = self.platform.laser_var['MOTORDEBUG']
        if (mode == 'cycletime') & (response != 0):
            response = int.from_bytes(response, "big")
            # you measure 180 degrees
            if response != 0:
                response = round((clock/(response*2)*60))
        elif (mode == 'PIcontrol'):
            degreecnt = int.from_bytes(response[(WORD_BYTES-2):],
                                       "big",
                                       signed=False)
            if degreecnt != 0:
                speed = (clock/(degreecnt*180*2)*60)
            else:
                speed = 0
            delay = int.from_bytes(response[:(WORD_BYTES-2)],
                                   "big",
                                   signed=True)
            response = [degreecnt, delay]
        elif (mode == 'anglecounter'):
            degreecnt = int.from_bytes(response, "big")
            if degreecnt != 0:
                response = (clock/(degreecnt*180*2)*60)
            else:
                response = 0
        else:
            response = int.from_bytes(response, "big")

        if not isinstance(response,
                          list):
            return [response]
        else:
            return response

    def get_state(self, data=None):
        '''retrieves the state of the FPGA as dictionary

        data: string to decode to state, if None data is retrieved from FPGA

        dictionary with the following keys
          parsing: True if commands are executed
          mem_full: True if memory is full
          error: True if an error state is reached by any of
                 the submodules
          x, y, z:            state of motor endswitches
          photodiode_trigger: True if photodiode is triggered during last
                              rotation of prism
          synchronized: True if laserhead is synchronized by photodiode
        '''
        if data is None:
            command = [COMMANDS.READ] + WORD_BYTES*[0]
            data = (yield from self.send_command(command))

        dct = {}
        # 9 bytes are returned
        # the state is decoded from byte 7 and 8, i.e. -2 and -1
        bits = "{:08b}".format(data[-1])
        dct['parsing'] = int(bits[STATE.PARSING])
        dct['error'] = int(bits[STATE.ERROR])
        dct['mem_full'] = int(bits[STATE.FULL])
        bits = "{:08b}".format(data[-2])
        mapping = list(self.platform.stepspermm.keys())
        for i in range(self.platform.motors):
            dct[mapping[i]] = int(bits[i])
        dct['photodiode_trigger'] = int(bits[self.platform.motors])
        dct['synchronized'] = int(bits[self.platform.motors+1])
        return dct

    @property
    def position(self):
        '''retrieves position from FPGA and updates internal position

           position is stored on the FPGA in steps
           position is stored on object in mm

           return positions as np.array in mm
                  order is [x, y, z]
        '''
        command = [COMMANDS.POSITION] + WORD_BYTES*[0]
        for i in range(self.platform.motors):
            read_data = (yield from self.send_command(command))
            self._position[i] = unpack('!q', read_data[1:])[0]
        # step --> mm
        self._position = (self._position /
                          np.array(list(self.platform.stepspermm.values())))
        return self._position

    @property
    def enable_steppers(self):
        '''returns 1 if steppers are enabled, 0 otherwise

        The enable pin for the stepper drivers is not routed via FPGA.
        The enable pin is low if enabled.
        Enabled stepper motor do not move if the FPGA is
        not parsing instructions from FIFO.
        '''
        return not self.enable.value

    @enable_steppers.setter
    def enable_steppers(self, val):
        '''set enable pin stepper motor drivers and parsing FIFO buffer by FPGA

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
    def laser_current(self):
        '''return laser current per channel as integer

        both channels have the same current
        integer ranges from 0 to 255 where
        0 no current and 255 full driver current
        '''
        if upython:
            data = bytearray(1)
            self.bus.recv(data)
        else:
            data = self.bus.read_byte_data(self.platform.ic_address,
                                           0)
        return data

    @laser_current.setter
    def laser_current(self, val):
        '''sets maximum laser current of laser driver per channel

        This does not turn on or off the laser. Laser is set
        to this current if pulsed. Laser current is set by enabling
        one or two channels. Second by setting a value between
        0-255 at the laser driver chip for the laser current. The laser
        needs a minimum current.
        '''
        if val < 0 or val > 150:
            # 255 kills laser at single channel
            raise Exception('Invalid or too high laser current')
        if upython:
            self.bus.mem_write(val, self.platform.ic_address)
        else:
            self.bus.write_byte_data(self.platform.ic_address, 0, val)

    def set_parsing(self, value):
        '''enables or disables parsing of FIFO by FPGA

        val -- True   FPGA parses FIFO
               False  FPGA does not parse FIFO
        '''
        assert type(value) == bool
        if value:
            command = [COMMANDS.START]
        else:
            command = [COMMANDS.STOP]
        command += WORD_BYTES*[0]
        return (yield from self.send_command(command))

    def home_axes(self, axes, speed=None, displacement=-200):
        '''home given axes, i.e. [1,0,1] homes x, z and not y

        axes         -- list with axes to home
        speed        -- speed in mm/s used to home
        displacement -- displacement used to touch home switch
        '''
        assert len(axes) == self.platform.motors
        dist = np.array(axes)*np.array([displacement]*self.platform.motors)
        yield from self.gotopoint(position=dist.tolist(),
                                  speed=speed, absolute=False)

    # TODO: this is strange, should it be here
    #       on the board steps and count is stored
    #       you could move this to spline_coefficients
    #       the flow over method of a certain bit comes from beagleg
    def steps_to_count(self, steps):
        '''compute count for a given number of steps

        steps  -- motor moves in small steps

        Shift is needed as two ticks per step are required
        You need to count slightly over the threshold. That is why
        +1 is added.
        '''
        bitshift = bit_shift(self.platform)
        count = (steps << (1+bitshift))+(1 << (bitshift-1))
        return count

    def gotopoint(self, position, speed=None, absolute=True):
        '''move machine to position or with displacement at constant speed

        Axes are moved independently to simplify the calculation.
        The move is carried out as a first order spline, i.e. only velocity.

        position     -- list with position or displacement in mm for each motor
        speed        -- list with speed in mm/s, if None default speeds used
        absolute     -- True if position, False if displacement
        '''
        assert len(position) == self.platform.motors
        if speed is not None:
            assert len(speed) == self.platform.motors
        else:
            speed = [10]*self.platform.motors
        # conversions to steps / count give rounding errors
        # minimized by setting speed to integer
        speed = np.absolute(np.array(speed))
        displacement = np.array(position)
        if absolute:
            # TODO: position machine should be in line with self._position
            #       which to pick?
            displacement -= self._position

        homeswitches_hit = [0]*len(position)
        for idx, disp in enumerate(displacement):
            if disp == 0:
                # no displacement, go to next axis
                continue
            # Time needed for move
            #    unit oscillator ticks (times motor position is updated)
            time = abs(disp/speed[idx])
            ticks_total = (time*MOTORFREQ).round().astype(int)
            # mm -> steps
            steps_per_mm = list(self.platform.stepspermm.values())[idx]
            speed_steps = int(round(speed[idx] * steps_per_mm*np.sign(disp)))
            speed_cnts = self.steps_to_count(speed_steps)/MOTORFREQ
            velocity = np.zeros_like(speed).astype('int64')
            velocity[idx] = speed_cnts

            if self.test:
                (yield from self.set_parsing(True))
            else:
                self.set_parsing(True)
            while ticks_total > 0:
                ticks_move = \
                     MOVE_TICKS if ticks_total >= MOVE_TICKS else ticks_total
                # execute move and retrieve if switch is hit
                switches_hit = (yield from self.spline_move(int(ticks_move),
                                                            velocity.tolist()))
                ticks_total -= ticks_move
                # move is aborted if home switch is hit and
                # velocity is negative
                cond = (switches_hit[idx] == 1) & (np.sign(disp) < 0)
                if cond:
                    break
        # update internally stored position
        self._position += displacement
        # set position to zero if home switch hit
        self._position[homeswitches_hit == 1] = 0

    def send_command(self, command, blocking=False):
        '''writes command to spi port

        blocking  --  try again if memory is full
        returns bytearray with length equal to data sent
        '''
        def send_command(command):
            assert len(command) == WORD_BYTES+COMMAND_BYTES
            if self.test:
                data = (yield from self.spi_exchange_data(command))
            else:
                data = (self.spi_exchange_data(command))
            return data
        if blocking:
            trials = 0
            while True:
                trials += 1
                data = (yield from send_command(command))
                state = (yield from self.get_state(data))
                if state['error']:
                    raise Exception("Error detected on FPGA")
                if not state['mem_full']:
                    break
                if trials > self.maxtrials:
                    raise Memfull(f"Too many trials {trials} needed")
        else:
            data = (yield from send_command(command))
        return data

    def enable_comp(self, laser0=False, laser1=False,
                    polygon=False, synchronize=False):
        '''enable components

        FPGA does need to be parsing FIFO
        These instructions are executed directly.

        laser0   -- True enables laser channel 0
        laser1   -- True enables laser channel 1
        polygon  -- False enables polygon motor
        '''
        laser0, laser1, polygon = (int(bool(laser0)),
                                   int(bool(laser1)),
                                   int(bool(polygon)))
        synchronize = int(bool(synchronize))
        data = ([COMMANDS.WRITE] + [0]*(WORD_BYTES-2) +
                [int(f'{synchronize}{polygon}{laser1}{laser0}', 2)]
                + [INSTRUCTIONS.WRITEPIN])
        yield from self.send_command(data, blocking=True)

    def spline_move(self, ticks, coefficients):
        '''write spline move instruction with ticks and coefficients to FIFO

        If you have 2 motors and execute a second order spline
        You send 4 coefficients.
        If the controller supports a third order spline,
        remaining coefficients are padded as zero.
        User needs to submit all coefficients up to highest order used.

        ticks           -- number of ticks in move, integer
        coefficients    -- coefficients for spline move per axis, list

        returns array with zero if home switch is hit
        '''
        platform = self.platform
        # maximum allowable ticks is move ticks,
        # otherwise counters overflow in FPGA
        assert ticks <= MOVE_TICKS
        assert len(coefficients) % platform.motors == 0
        write_byte = COMMANDS.WRITE.to_bytes(1, 'big')
        move_byte = INSTRUCTIONS.MOVE.to_bytes(1, 'big')
        commands = [write_byte +
                    ticks.to_bytes(7, 'big') + move_byte]
        # check max order given by caller of function
        max_coeff_order = (len(coefficients)//platform.motors)
        # prepare commands
        for motor in range(platform.motors):
            for degree in range(platform.poldegree):
                # set to zero if coeff not provided by caller
                if degree > max_coeff_order-1:
                    coeff = 0
                else:
                    idx = degree+motor*max_coeff_order
                    coeff = coefficients[idx]
                data = coeff.to_bytes(8, 'big', signed=True)
                commands += [write_byte + data]
        # send commands to FPGA
        for command in commands:
            data_out = (yield from self.send_command(command,
                                                     blocking=True))
            state = (yield from self.get_state(data_out))
        axes_names = list(platform.stepspermm.keys())
        return np.array([state[key] for key in axes_names])

    def spi_exchange_data(self, data):
        '''writes data to peripheral

        data  --  command followed with word
                     list of multiple bytes

        returns bytearray with length equal to data sent
        '''
        assert len(data) == (COMMAND_BYTES + WORD_BYTES)
        self.chip_select.off()
        # spidev changes values passed to it
        if not upython:
            from copy import deepcopy
            datachanged = deepcopy(data)
            response = bytearray(self.spi.xfer2(datachanged))
        else:
            response = bytearray(data)
            self.spi.write_readinto(data, response)
        self.chip_select.on()
        return response

    def writeline(self, bitlst, stepsperline=1, direction=0):
        '''write bits to FIFO

           bit list      bits which are written to substrate
                         at the moment laser can only be on of off
                         if bitlst is empty stop command is sent
           stepsperline  stepsperline, should be greater than 0
                         if you don't want to move simply disable motor
           direction     motor direction of scanning axis
        '''
        bytelst = self.bittobytelist(bitlst, stepsperline, direction)
        write_byte = COMMANDS.WRITE.to_bytes(1, 'big')
        for i in range(0, len(bytelst), 8):
            lst = bytelst[i:i+8]
            lst.reverse()
            data = write_byte + bytes(lst)
            (yield from self.send_command(data, blocking=True))

    def bittobytelist(self, bitlst, stepsperline=1,
                      direction=0, bitorder='little'):
        '''converts bitlst to bytelst

           bit list      bits which are written to substrate
                         at the moment laser can only be on of off
                         if bitlst is empty stop command is sent
           stepsperline  stepsperline, should be greater than 0
                         if you don't want to move simply disable motor
           direction     motor direction of scanning axis
        '''
        # the halfperiod is sent over
        # this is the amount of ticks in half a cycle of
        # the motor
        # watch out for python "banker's rounding"
        # sometimes target might not be equal to steps
        bits = self.laser_params['BITSINSCANLINE']
        halfperiod = int((bits-1) // (stepsperline*2))
        if (halfperiod < 1):
            raise Exception("Steps per line cannot be achieved")
        # TODO: is this still an issue?
        # you could check as follows
        # steps = self.laser_params['TICKSINFACET']/(halfperiod*2)
        #    print(f"{steps} is actual steps per line")
        direction = [int(bool(direction))]

        def remainder(bytelst):
            rem = (len(bytelst) % WORD_BYTES)
            if rem > 0:
                res = WORD_BYTES - rem
            else:
                res = 0
            return res
        if len(bitlst) == 0:
            bytelst = [INSTRUCTIONS.LASTSCANLINE]
            bytelst += remainder(bytelst)*[0]
        else:
            assert len(bitlst) == self.laser_params['BITSINSCANLINE']
            assert max(bitlst) <= 1
            assert min(bitlst) >= 0
            bytelst = [INSTRUCTIONS.SCANLINE]
            halfperiodbits = [int(i) for i in bin(halfperiod)[2:]]
            halfperiodbits.reverse()
            assert len(halfperiodbits) < 56
            bytelst += np.packbits(direction+halfperiodbits,
                                   bitorder=bitorder).tolist()
            bytelst += remainder(bytelst)*[0]
            bytelst += np.packbits(bitlst, bitorder=bitorder).tolist()
            bytelst += remainder(bytelst)*[0]
        return bytelst
