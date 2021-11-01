from struct import unpack
from time import sleep
from copy import deepcopy

import numpy as np

import hexastorm.lasers as lasers
from hexastorm.constants import (INSTRUCTIONS, COMMANDS, MOTORFREQ, STATE,
                                 MOVE_TICKS, WORD_BYTES, bit_shift,
                                 COMMAND_BYTES)
from hexastorm.platforms import Firestarter, TestPlatform
import hexastorm.core as core


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
    def __init__(self, test=False):
        '''  test  -- True if behavior of FPGA is simulated
        '''
        if test:
            self.platform = TestPlatform
        else:
            from gpiozero import LED
            import spidev
            from smbus2 import SMBus
            self.platform = Firestarter()
            # IC bus used to set power laser
            self.bus = SMBus(self.platform.ic_dev_nr)
            # SPI to sent data to scanner
            self.spi = spidev.SpiDev()
            self.spi.open(0, 0)
            self.spi.mode = 1
            self.spi.max_speed_hz = round(1E6)
            self.chip_select = LED(8)
            # programs TMC2130 
            self.init_steppers()
            # stepper motor enable pin
            self.enable = LED(self.platform.enable_pin)
        self.test = test
        self.laser_params = lasers.params(self.platform)
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
           
           do_program  -- flashes the FPGA chip using fomu-flash
           verbose     -- prints output of Yosys, Nextpnr and icepack
        '''
        self.platform = Firestarter()
        self.platform.laser_var = self.laser_params
        self.platform.build(core.Dispatcher(self.platform),
                            do_program=do_program, 
                            verbose=verbose)

    def reset(self):
        'restart the FPGA by flipping the reset pin'
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
        dct ['mem_full'] = int(bits[STATE.FULL])
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
        from gpiozero import LED
        return not LED(self.platform.enable_pin).value

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
        return self.bus.read_byte_data(self.platform.ic_address, 0)

    @laser_current.setter
    def laser_current(self, val):
        '''sets maximum laser current of laser driver per channel
         
        This does not turn on or off the laser. Laser is set
        to this current if pulsed. Laser current is set by enabling 
        one or two channels. Second by setting a value between
        0-255 at the laser driver chip for the laser current. The laser
        needs a minimum current.
        '''
        if val < 0 or val > 255:
            raise Exception('Invalid laser current')
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

    #TODO: this is strange, should it be here?
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
        speed        -- list with speed in mm/s, if None default speeds are used
        absolute     -- True if position, False if displacement
        '''
        assert len(position) == self.platform.motors
        if speed is not None:
            assert len(speed) == self.platform.motors
        else:
            speed = [10]*self.platform.motors
        speed = np.array(speed)
        displacement = np.array(position)
        if absolute:
            displacement -= self._position
            
        for idx, _ in enumerate(position):
            if displacement[idx] == 0:
                # no displacement, go to next axis
                continue
            # Time needed for move
            #    unit oscillator ticks (times motor position is updated)
            time = np.array([np.absolute((displacement/speed))[idx]])
            ticks_total = (time*MOTORFREQ).round().astype(int)
            # select axis
            select = np.zeros_like(position, dtype='int64')
            select[idx] = 1
            # mm -> steps
            steps_per_mm = np.array(list(self.platform.stepspermm.values()))
            displacement_steps = \
            (displacement * steps_per_mm * select).round().astype('int64')
            

            def move_ticks(move_ticks):
                '''number of ticks in next segment
                
                If there are 15 ticks left and max per
                segment is 10 ticks, next segment is 
                10 ticks and 5 remain
                '''
                if move_ticks >= MOVE_TICKS:
                    return MOVE_TICKS
                else:
                    return move_ticks
            move_ticks_v = np.vectorize(move_ticks)

            ticks_remain = np.copy(ticks_total)
            total_moved_steps = np.zeros_like(displacement_steps,
                                              dtype='int64')
            if self.test:
                (yield from self.set_parsing(True))
            else:
                self.set_parsing(True)
            while ticks_remain.sum() > 0:
                ticks_move = move_ticks_v(ticks_remain)
                ticks_remain -= ticks_move
                # steps -> count
                # this introduces a rounding error
                move_steps = (displacement_steps*(ticks_move/ticks_total)).round().astype('int64')
                total_moved_steps += move_steps
                # due to rounding we could move more than wanted
                # correct if this is the case
                cond = (np.absolute(total_moved_steps)
                        > np.absolute(displacement_steps))
                if cond.any():
                    move_steps[cond] -= (
                     (total_moved_steps-displacement_steps)[cond])
                    total_moved_steps[cond] = displacement_steps[cond]
                # prepare arguments for spline move
                cnts = self.steps_to_count(move_steps)
                velocity = (cnts/ticks_move).round().astype('int64')
                # execute move and retrieve if switch is hit
                homeswitches_hit = (yield from self.spline_move(int(ticks_move),
                                                                velocity.tolist()))
                # move is aborted if home switch is hit and
                # velocity is negative
                cond = (homeswitches_hit == 0) | (velocity > 0)
                if displacement_steps[cond].sum() == 0:
                    break
            # update internally stored position
            dist_mm = total_moved_steps / steps_per_mm
            self._position += dist_mm
            # set position to zero if home switch hit
            self._position[homeswitches_hit == 1] = 0

    def send_command(self, data):
        assert len(data) == WORD_BYTES+COMMAND_BYTES
        if self.test:
            data = (yield from self.spi_exchange_data(data))
        else:
            data = (self.spi_exchange_data(data))
        return data

    def enable_comp(self, laser0=False, laser1=False,
                    polygon=False, synchronize=False):
        '''enable components

        FPGA should be parsing FIFO otherwise there
        is no result

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
        yield from self.send_command(data)

    def spline_move(self, ticks, coefficients, maxtrials=1E5):
        '''write spline move instruction with ticks and coefficients to FIFO

        If you have 2 motors and execute a second order spline
        You send 4 coefficients.
        If the controller supports a third order spline,
        remaining coefficients are padded as zero.
        User needs to submit all coefficients up to highest order used.

        ticks           -- number of ticks in move, integer
        coefficients    -- coefficients for spline move per axis, list
        maxtrials       -- max number of communication trials

        returns array with zero if home switch is hit
        '''
        platform = self.platform
        if self.test:
            maxtrials = 10
        # maximum allowable ticks is move ticks, 
        # otherwise counters overflow in FPGA
        assert ticks <= MOVE_TICKS
        assert len(coefficients)%platform.motors == 0

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
            trials = 0
            while True:
                data_out = (yield from self.send_command(command))
                state = (yield from self.get_state(data_out))
                trials += 1
                if state['error']:
                    raise Exception("Error detected on FPGA")
                if not state['mem_full']:
                    break
                if trials > maxtrials:
                    raise Memfull(f"Too many trials {trials} needed")
        axes_names = list(platform.stepspermm.keys())
        return np.array([state[key] for key in axes_names])

    def spi_exchange_data(self, data):
        '''writes data to peripheral
        
        data  --  COMMAND byte followed with word BYTES
                  
        returns bytearray with length equal to data sent
        '''
        assert len(data) == (COMMAND_BYTES + WORD_BYTES)
        self.chip_select.off()
        # spidev changes values passed to it
        datachanged = deepcopy(data)
        response = bytearray(self.spi.xfer2(datachanged))
        self.chip_select.on()
        return response

    def writeline(self, bitlst, stepsperline=1, direction=0, maxtrials=1E5):
        bytelst = self.bittobytelist(bitlst, stepsperline, direction)
        write_byte = COMMANDS.WRITE.to_bytes(1, 'big')
        # TODO: merge write line with send commands
        if self.test:
            maxtrials = 10
        for i in range(0, len(bytelst), 8):
            trials = 0
            lst = bytelst[i:i+8]
            lst.reverse()
            data = write_byte + bytes(lst)
            while True:
                trials += 1
                data_out = (yield from self.send_command(data))
                if not (yield from self.get_state(data_out))['mem_full']:
                    break
                if trials > maxtrials:
                    raise Memfull("Too many trials needed")

    def bittobytelist(self, bitlst, stepsperline=1,
                      direction=0, bitorder='little'):
        '''converts bitlst to bytelst

           bit list      set laser on and off
                         if bitlst is empty stop command is sent
           stepsperline  stepsperline, should be greater than 0
                         if you don't want to move simply disable motor
           direction     scanning direction
        '''
        # NOTE: the halfperiod is sent over
        #       this is the amount of ticks in half a cycle of
        #       the motor
        halfperiod = round((self.laser_params['TICKSINFACET']-1)
                           / (stepsperline*2))
        # watch out for python "banker's rounding"
        # sometimes target might not be equal to steps
        # this is ignored for now
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
