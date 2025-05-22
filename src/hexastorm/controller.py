from struct import unpack
from time import sleep
import sys
import logging

try:
    import numpy as np
except ImportError or ModuleNotFoundError:
    from ulab import numpy as np

from . import ulabext
from .constants import (
    COMMAND_BYTES,
    COMMANDS,
    INSTRUCTIONS,
    MOTORFREQ,
    MOVE_TICKS,
    STATE,
    WORD_BYTES,
    bit_shift,
    params,
)

logger = logging.getLogger(__name__)


def executor(func):
    """convert Amaranth style function in normal callable function

    amaranth introduces generators in call chain
    when using Amaranth, i.e. python, this returns a generator
    in the case of python, the generator is consumed
    """
    def inner(*args, **kwargs):
        try:
            next(func(*args, **kwargs))
        # function is regular and not a generator
        # decorator is not needed
        except TypeError:
            return func(*args, **kwargs)
        except StopIteration as e:
            return e.value
    return inner


class Memfull(Exception):
    """SRAM memory of FPGA, i.e. FIFO, is full

    Exception is raised when the memory is full.
    """
    pass


class Host:
    """Class to interact with FPGA"""

    def __init__(self, platform=None, micropython=False):
        """platform  -- object which has gateware settings
        only passed to controller if virtual
        test is executed. Needed in lasers.py
        as each test here has a slightly
        different TestPlatform
        micropython -- if true object uses libraries suited
        for micropython
        """
        self.steppers_initialized = False

        if sys.implementation.name == "micropython":
            self.micropython = True
        else:
            self.micropython = micropython
        self.max_attempts = 1
       
        if platform is None:
            self.test = False
            # case micropython
            if self.micropython:
                self.max_attempts = 3
                self.init_micropython()
            # case raspberry
            else:
                from gpiozero import LED
                from .platforms import Firestarter

                import spidev
                from smbus2 import SMBus

                self.reset_pin = LED(self.platform.reset_pin)
                self.platform = Firestarter(micropython=False)
                # IC bus used to set power laser
                self.bus = SMBus(self.platform.ic_dev_nr)
                # SPI to sent data to scanner
                self.spi = spidev.SpiDev()
                self.spi.open(*self.platform.spi_dev)
                self.spi.mode = 1
                self.spi.max_speed_hz = round(1e6)
                self.fpga_select = LED(self.platform.chip_select)
                # drivers are now in standalone mode
                # self.init_steppers()
                # stepper motor enable pin
                self.enable = LED(self.platform.enable_pin)
                self.enable.on()
        else:
            self.platform = platform
            self.test = True
        # maximum number of times tried to write to FIFO
        # if memory is full
        self.maxtrials = 10 if self.test else 1e5
        self.laser_params = params(self.platform)
        try:
            self._position = np.array([0] * self.platform.motors, dtype=float)
        # different syntax in micropython
        except TypeError:
            self._position = np.array(
                [0] * self.platform.motors, dtype=np.float
            )

    def init_micropython(self):
        from .constants import platform as platformmicro
        from machine import Pin, SPI, I2C

        self.platform = platformmicro(micropython=True)
        self.reset_pin = Pin(self.platform.reset_pin, Pin.OUT)
        self.reset_pin.value(1)
        self.bus = I2C(
            scl=Pin(self.platform.scl), sda=Pin(self.platform.sda)
        )
        # hardware SPI works partly, set speed to 3e6
        # return bytes give issue in retrieving position
        self.spi = SPI(2,
            baudrate=self.platform.baudrate,
            polarity=0,
            phase=self.platform.phase,
            sck=Pin(self.platform.sck, Pin.OUT),
            mosi=Pin(self.platform.mosi, Pin.OUT),
            miso=Pin(self.platform.miso, Pin.IN),
        )
        # keep for hardware SPI
        self.spi.deinit()
        self.spi.init()
        self.flash_select = Pin(self.platform.flash_cs, Pin.OUT)
        self.flash_select.value(1)
        self.fpga_select = Pin(self.platform.fpga_cs, Pin.OUT)
        # stepper motor enable pin
        self.enable = Pin(self.platform.enable_pin, Pin.OUT)
        self.init_steppers()

    def init_steppers(self, current=100):
        """ steppers are configured for TMC2209
            current in mA
        """
        if self.micropython and not self.steppers_initialized:
            from tmc.stepperdriver import TMC_2209
            from tmc.uart import ConnectionFail
            failed = False
            for key, value in self.platform.tmc2209.items():
                try:
                    tmc = TMC_2209(pin_en=38, mtr_id=value)
                    tmc.setDirection_reg(False)
                    tmc.setVSense(True)
                    tmc.setCurrent(current)
                    tmc.setIScaleAnalog(True)
                    tmc.setInterpolation(True)
                    tmc.setSpreadCycle(False)
                    tmc.setMicrosteppingResolution(16)
                    tmc.setInternalRSense(False)
                    tmc.setMotorEnabled(False)
                except ConnectionFail:
                    failed = True
                    logging.debug(f"Cannot connect to stepper motor {key} axis")
            if not failed:
                self.steppers_initialized = True


    def flash_fpga(self, filename):
        if not self.micropython:
            raise Exception("Only supported for micropython.")
        from machine import SoftSPI, Pin
        from winbond import W25QFlash
        self.reset_pin.value(0)
        self.flash_select.value(1)
        sleep(1)
        # can't get hardware spi working with memory
        spi = SoftSPI(
                  polarity=0,
                  phase =1,
                  sck=Pin(self.platform.sck),
                  mosi=Pin(self.platform.mosi),
                  miso=Pin(self.platform.miso))
        spi.deinit()
        spi.init()
        f = W25QFlash(
            spi=spi,
            cs=self.flash_select,
            baud=self.platform.baudrate,
            software_reset=True,
        )

        self.reset_pin.value(0)
        buffsize = f.BLOCK_SIZE
        # if dest.endswith("/"):  # minimal way to allow
        #    dest = "".join((dest, source.split("/")[-1]))  # cp /sd/file /fl_ext/

        with open(filename, "rb") as infile:
            blocknum = 0
            while True:
                buf = infile.read(buffsize)
                logging.info(f" Writing {blocknum}.")
                f.writeblocks(blocknum, buf)
                if len(buf) < buffsize:
                    logging.info(f"Final block {blocknum}")
                    break
                else:
                    blocknum += 1
        self.reset()
        logging.info("flashed fpga")


    def build(self, do_program=True, verbose=True, mod="all"):
        """builds the FPGA code using amaranth HDL, Yosys, Nextpnr and icepack

        do_program  -- flashes the FPGA chip using fomu-flash,
                       resets aftwards
        verbose     -- prints output of Yosys, Nextpnr and icepack
        """
        if self.micropython:
            logging.info("Micropython cannot update binary, using stored one")
        else:
            from .core import Dispatcher
            from .motor import Driver
            from .platforms import Firestarter

            if mod == "all":
                module = Dispatcher(self.platform)
            elif mod == "motor":
                module = Driver(self.platform, top=True)
            else:
                raise Exception(f"Print building {mod} is not supported.")
            self.platform = Firestarter()
            self.platform.laser_var = self.laser_params
            self.platform.build(
                module,
                do_program=do_program,
                verbose=verbose,
            )
        if do_program:
            self.reset()

    def reset(self):
        "restart the FPGA by flipping the reset pin"
        from machine import Pin
        # free all lines
        sck = Pin(12, Pin.IN)
        mosi = Pin(13, Pin.IN)
        miso = Pin(11, Pin.IN)
        slct = Pin(10, Pin.IN)
        self.reset_pin.value(0)
        sleep(1)
        self.reset_pin.value(1)
        sleep(1)
        self.init_micropython()
        command = [0] * (WORD_BYTES + COMMAND_BYTES)
        command = bytearray(command)
        response = bytearray(command)
        self.fpga_select.value(0)
        self.spi.write_readinto(command, response)
        self.fpga_select.value(1)

    def get_motordebug(self, blocking=False):
        """retrieves the motor debug word

        This is used to debug the PI controller and
        set the correct setting for the Hall interpolation

        blocking   -- checks if memory is full, only needed for
                      a build with all modules
        """
        command = [COMMANDS.DEBUG] + WORD_BYTES * [0]
        response = (yield from self.send_command(command, blocking=blocking))[
            1:
        ]

        clock = int(self.platform.clks[self.platform.hfosc_div] * 1e6)
        mode = self.platform.laser_var["MOTORDEBUG"]

        def cntcnv(cnt):
            if cnt != 0:
                speed = (
                    clock
                    / (cnt * 4 * self.platform.laser_var["MOTORDIVIDER"])
                    * 60
                )
            else:
                speed = 0
            return speed

        if (mode == "cycletime") & (response != 0):
            response = int.from_bytes(response, "big")
            # you measure 180 degrees
            if response != 0:
                response = round((clock / (response * 2) * 60))
        elif mode == "PIcontrol":
            cnt = int.from_bytes(
                response[(WORD_BYTES - 2) :], "big", signed=False
            )
            speed = cntcnv(cnt)
            duty = int.from_bytes(
                response[: (WORD_BYTES - 2)], "big", signed=True
            )
            response = [speed, duty]
        elif mode == "ticksinfacet":
            cnt = int.from_bytes(
                response[(WORD_BYTES - 2) :], "big", signed=False
            )
            speed = cntcnv(cnt)
            cntdiode = int.from_bytes(
                response[: (WORD_BYTES - 2)], "big", signed=False
            )
            speedd = cntcnv(cntdiode)
            response = [speed, speedd]
        else:
            response = int.from_bytes(response, "big")

        if not isinstance(response, list):
            return [response]
        else:
            return response


    def get_state(self, data=None):
        """retrieves the state of the FPGA as dictionary

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
        """
        if data is None:
            command = [COMMANDS.READ] + WORD_BYTES * [0]
            data = yield from self.send_command(command)

        dct = {}
        # 9 bytes are returned
        # the state is decoded from byte 7 and 8, i.e. -2 and -1
        byte1 = data[-1]  # Last byte

        dct["parsing"] = (byte1 >> (7-STATE.PARSING)) & 1
        dct["error"] = (byte1 >> (7-STATE.ERROR)) & 1
        dct["mem_full"] =  (byte1 >> (7-STATE.FULL)) & 1

        byte2 = data[-2]  # Second to last byte
        mapping = list(self.platform.stepspermm.keys())
        for i in range(self.platform.motors):
            dct[mapping[i]] = (byte2 >> (7-i)) & 1
        dct["photodiode_trigger"] = (byte2 >> (7-self.platform.motors)) & 1
        dct["synchronized"] = (byte2 >> (7-self.platform.motors - 1)) & 1
        return dct

    @property
    def position(self):
        """retrieves position from FPGA and updates internal position

        position is stored on the FPGA in steps
        position is stored on object in mm

        return positions as np.array in mm
               order is [x, y, z]
        """
        command = [COMMANDS.POSITION] + WORD_BYTES * [0]
        for i in range(self.platform.motors):
            read_data = yield from self.send_command(command)
            self._position[i] = unpack("!q", read_data[1:])[0]
            # code below is not portable between python and micropython
            # python requires signed=True, micropython does not accep this
            # overflow error can be generated, if you go below 0
            # overflow is created during the division,
            # it's assumed position cannot be negative.
            # self._position[i] = int.from_bytes(read_data[1:9], 'big', True)
        # step --> mm
        self._position = self._position / np.array(
            list(self.platform.stepspermm.values())
        )
        return self._position

    @property
    def enable_steppers(self):
        """returns 1 if steppers are enabled, 0 otherwise

        The enable pin for the stepper drivers is not routed via FPGA.
        The enable pin is low if enabled.
        Enabled stepper motor do not move if the FPGA is
        not parsing instructions from FIFO.
        """
        return not self.enable.value

    @enable_steppers.setter
    def enable_steppers(self, val):
        """set enable pin stepper motor drivers and parsing FIFO buffer by FPGA

        val -- boolean, True enables steppers
        """
        if val:
            self.enable.off() if not self.micropython else self.enable.value(0)
        else:
            self.enable.on() if not self.micropython else self.enable.value(1)

    @property
    def laser_current(self):
        """return laser current per channel as integer

        both channels have the same current
        integer ranges from 0 to 255 where
        0 no current and 255 full driver current
        """
        if self.micropython:
            data = list(self.bus.readfrom_mem(self.platform.ic_address, 0, 1))[
                0
            ]
        else:
            data = self.bus.read_byte_data(self.platform.ic_address, 0)
        return data

    @laser_current.setter
    def laser_current(self, val):
        """sets maximum laser current of laser driver per channel

        This does not turn on or off the laser. Laser is set
        to this current if pulsed. Laser current is set by enabling
        one or two channels. Second by setting a value between
        0-255 at the laser driver chip for the laser current. The laser
        needs a minimum current.
        """
        if val < 0 or val > 150:
            # 255 kills laser at single channel
            raise Exception("Invalid or too high laser current")
        if self.micropython:
            self.bus.writeto_mem(self.platform.ic_address, 0, bytes([val]))
        else:
            self.bus.write_byte_data(self.platform.ic_address, 0, val)


    def set_parsing(self, value):
        """enables or disables parsing of FIFO by FPGA

        val -- True   FPGA parses FIFO
               False  FPGA does not parse FIFO
        """
        assert isinstance(value, bool)
        if value:
            command = [COMMANDS.START]
        else:
            command = [COMMANDS.STOP]
        command += WORD_BYTES * [0]
        return (yield from self.send_command(command))

    def home_axes(self, axes, speed=None, displacement=-200):
        """home given axes, i.e. [1,0,1] homes x, z and not y

        axes         -- list with axes to home
        speed        -- speed in mm/s used to home
        displacement -- displacement used to touch home switch
        """
        assert len(axes) == self.platform.motors
        dist = np.array(axes) * np.array([displacement] * self.platform.motors)
        yield from self.gotopoint(
            position=dist.tolist(), speed=speed, absolute=False
        )

    # TODO: this is strange, should it be here
    #       on the board steps and count is stored
    #       you could move this to spline_coefficients
    #       the flow over method of a certain bit comes from beagleg
    def steps_to_count(self, steps):
        """compute count for a given number of steps

        steps  -- motor moves in small steps

        Shift is needed as two ticks per step are required
        You need to count slightly over the threshold. That is why
        +1 is added.
        """
        bitshift = bit_shift(self.platform)
        count = (steps << (1 + bitshift)) + (1 << (bitshift - 1))
        return count

    def gotopoint(self, position, speed=None, absolute=True):
        """move machine to position or with displacement at constant speed

        Axes are moved independently to simplify the calculation.
        The move is carried out as a first order spline, i.e. only velocity.

        position     -- list with position or displacement in mm for each motor
        speed        -- list with speed in mm/s, if None default speeds used
        absolute     -- True if position, False if displacement
        """
        (yield from self.set_parsing(True))
        assert len(position) == self.platform.motors
        if speed is not None:
            assert len(speed) == self.platform.motors
        else:
            speed = [10] * self.platform.motors
        # conversions to steps / count gives rounding errors
        # minimized by setting speed to integer
        speed = abs(np.array(speed))
        displacement = np.array(position)
        if absolute:
            # TODO: position machine should be in line with self._position
            #       which to pick?
            displacement -= self._position

        homeswitches_hit = [0] * len(position)
        for idx, disp in enumerate(displacement):
            if disp == 0:
                # no displacement, go to next axis
                continue
            # Time needed for move
            #    unit oscillator ticks (times motor position is updated)
            time = abs(disp / speed[idx])
            ticks_total = round(time * MOTORFREQ)
            # mm -> steps
            steps_per_mm = list(self.platform.stepspermm.values())[idx]
            speed_steps = int(
                round(speed[idx] * steps_per_mm * ulabext.sign(disp))
            )
            velocity = [0] * len(speed)
            velocity[idx] = self.steps_to_count(speed_steps) // MOTORFREQ
            (yield from self.set_parsing(True))

            while ticks_total > 0:
                ticks_move = (
                    MOVE_TICKS if ticks_total >= MOVE_TICKS else ticks_total
                )
                # execute move and retrieve if switch is hit
                switches_hit = yield from self.spline_move(
                    int(ticks_move), velocity
                )
                ticks_total -= ticks_move
                # move is aborted if home switch is hit and
                # velocity is negative
                cond = (switches_hit[idx] == 1) & (ulabext.sign(disp) < 0)
                if cond:
                    break
        # update internally stored position
        self._position += displacement
        # set position to zero if home switch hit
        self._position[homeswitches_hit == 1] = 0
        # TODO: you enable parsing but don't disable it
        #       this would require a wait or maybe it should be enabled on

    def send_command(self, command, blocking=False):
        """writes command to spi port

        blocking  --  try again if memory is full
        returns bytearray with length equal to data sent
        """
        assert len(command) == WORD_BYTES + COMMAND_BYTES
        def send_command(command, response):
            # caller arguments are bytearrays and mutable
            if self.test:
                # function is created in Amaranth HDL
                response[:] = (yield from self.spi_exchange_data(command))
            else:
                self.fpga_select.value(0)
                self.spi.write_readinto(command, response)
                self.fpga_select.value(1)
        command = bytearray(command)
        response = bytearray(command)

        if blocking:
            trials = 0
            while True:
                trials += 1
                yield from send_command(command, response)
                byte1 = response[-1]  # can't rely on self.get_state (needs speed!)
                if (byte1 >> (7-STATE.ERROR)) & 1:
                    if self.micropython:
                        # lower SPI speed, change FPGA implementation
                        # you cannot recover from this
                        self.reset()
                    raise Exception("Error detected on FPGA")
                if not ((byte1 >> (7-STATE.FULL)) & 1):
                    break
                if trials > self.maxtrials:
                    raise Memfull(f"Too many trials {trials} needed")
        else:
            yield from send_command(command, response)

        return response

    def enable_comp(
        self, laser0=False, laser1=False, polygon=False, synchronize=False, 
        singlefacet=False
    ):
        """enable components
        
        FPGA does need to be parsing FIFO
        These instructions are executed directly.

        laser0   -- True enables laser channel 0
        laser1   -- True enables laser channel 1
        polygon  -- False enables polygon motor
        synchronize -- Enable synchronization
        singlefacet -- Enable singlefacet
        """
        laser0, laser1, polygon = (
            int(bool(laser0)),
            int(bool(laser1)),
            int(bool(polygon)),
        )
        synchronize = int(bool(synchronize))
        singlefacet = int(bool(singlefacet))
        data = (
            [COMMANDS.WRITE]
            + [0] * (WORD_BYTES - 2)
            + [int(f"{singlefacet}{synchronize}{polygon}{laser1}{laser0}", 2)]
            + [INSTRUCTIONS.WRITEPIN]
        )
        yield from self.send_command(data, blocking=True)

    def spline_move(self, ticks, coefficients):
        """write spline move instruction with ticks and coefficients to FIFO

        If you have 2 motors and execute a second order spline
        You send 4 coefficients.
        If the controller supports a third order spline,
        remaining coefficients are padded as zero.
        User needs to submit all coefficients up to highest order used.

        ticks           -- number of ticks in move, integer
        coefficients    -- coefficients for spline move per axis, list

        returns array with zero if home switch is hit
        """
        platform = self.platform
        # maximum allowable ticks is move ticks,
        # otherwise counters overflow in FPGA
        assert ticks <= MOVE_TICKS
        assert len(coefficients) % platform.motors == 0
        write_byte = COMMANDS.WRITE.to_bytes(1, "big")
        move_byte = INSTRUCTIONS.MOVE.to_bytes(1, "big")
        commands = [write_byte + ticks.to_bytes(7, "big") + move_byte]
        # check max order given by caller of function
        max_coeff_order = len(coefficients) // platform.motors
        # prepare commands
        for motor in range(platform.motors):
            for degree in range(platform.poldegree):
                # set to zero if coeff not provided by caller
                if degree > max_coeff_order - 1:
                    coeff = 0
                else:
                    idx = degree + motor * max_coeff_order
                    coeff = coefficients[idx]
                if sys.implementation.name == 'micropython':
                    data = int(coeff).to_bytes(8, "big", True)
                else:
                    data = int(coeff).to_bytes(8, "big", signed=True)
                commands += [write_byte + data]
        # send commands to FPGA
        for command in commands:
            data_out = yield from self.send_command(command, blocking=True)
            state = yield from self.get_state(data_out)
        axes_names = list(platform.stepspermm.keys())
        return np.array([state[key] for key in axes_names])

    def writeline(self, bitlst, stepsperline=1, direction=0, repetitions=1):
        """projects given bitlst as line with laserhead

        bit list      bits which are written to substrate
                      at the moment laser can only be on or off
                      if bitlst is empty stop command is sent
        stepsperline  stepsperline, should be greater than 0
                      if you don't want to move simply disable motor
        repitition    multiple times line is repeated
        direction     motor direction of scanning axis
        """
        bytelst = self.bittobytelist(bitlst, stepsperline, direction)
        for _ in range(repetitions):
            cmdlst = self.bytetocmdlist(bytelst)
            for cmd in cmdlst:
                (yield from self.send_command(cmd, blocking=True))

    def bytetocmdlist(self, bytelst):
        cmdlist = []
        write_byte = COMMANDS.WRITE.to_bytes(1, "big")
        bytelst_len = len(bytelst)
        for i in range(0, bytelst_len, 8):
            lst = bytelst[i : i + 8]
            lst.reverse()
            data = write_byte + bytes(lst)
            cmdlist.append(data)
        return cmdlist

    def bittobytelist(
        self, bitlst, stepsperline=1, direction=0, bitorder="little"
    ):
        """converts bitlst to bytelst

        bit list      bits which are written to substrate
                      at the moment laser can only be on or off
                      if bitlst is empty stop command is sent
        stepsperline  stepsperline, should be greater than 0
                      if you don't want to move simply disable motor
        direction     motor direction of scanning axis
        """
        # the halfperiod is sent over
        # this is the amount of ticks in half a cycle of
        # the motor
        # watch out for python "banker's rounding"
        # sometimes target might not be equal to steps
        bits_in_scanline = self.laser_params["BITSINSCANLINE"]
        halfperiod = int((bits_in_scanline - 1) // (stepsperline * 2))
        if halfperiod < 1:
            raise Exception("Steps per line cannot be achieved")
        # TODO: is this still an issue?
        # you could check as follows
        # steps = self.laser_params['TICKSINFACET']/(halfperiod*2)
        #    print(f"{steps} is actual steps per line")
        direction_byte = [int(bool(direction))]



        def pad_with_zeros(bytelst):
            padding_length = (WORD_BYTES - len(bytelst) % WORD_BYTES) % WORD_BYTES
            bytelst.extend([0] * padding_length)

        if not len(bitlst):
            bytelst = [INSTRUCTIONS.LASTSCANLINE]
            pad_with_zeros(bytelst)
            return bytelst


        bytelst = [INSTRUCTIONS.SCANLINE]
        halfperiodbits = [int(i) for i in bin(halfperiod)[2:]]
        halfperiodbits.reverse()
        assert len(halfperiodbits) < 56
        bytelst.extend(list(
            ulabext.packbits(direction_byte + halfperiodbits, bitorder=bitorder)
        ))
        pad_with_zeros(bytelst)

        bitlst_len = len(bitlst)
        if bitlst_len == bits_in_scanline:
            bytelst.extend(ulabext.packbits(bitlst, bitorder=bitorder))
        elif bitlst_len == bits_in_scanline // 8:
            bytelst.extend(bitlst)
        else:
            raise Exception("Invalid line length")
        pad_with_zeros(bytelst)
        return bytelst
