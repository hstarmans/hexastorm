from struct import unpack
import sys
from time import sleep
import logging

from . import ulabext
from .config import Spi, PlatformConfig

if sys.implementation.name != "micropython":
    import numpy as np

    from .core import Dispatcher
    from .platforms import Firestarter
else:
    from machine import Pin, SPI, I2C, SoftSPI

    from tmc.uart import ConnectionFail
    from tmc.stepperdriver import TMC_2209
    from ulab import numpy as np
    from winbond import W25QFlash


logger = logging.getLogger(__name__)


class Memfull(Exception):
    """Raised when the FPGA SRAM (e.g. FIFO) is full.

    Exception is raised when the memory cannot accept more data.
    """

    pass


class BaseHost:
    """
    Host interface to interact with the FPGA.

    Several implementation exist such as test and micropython.

    Args:
        test (bool): If True, runs in test mode with virtual FPGA platform.

    In test mode, the object uses mock settings and disables MicroPython-specific code.
    """

    def __init__(self, test=False):
        self.test = test
        self.cfg = PlatformConfig(self.test)

    @property
    async def position(self):
        """retrieves position from FPGA and updates internal position

        position is stored on the FPGA in steps
        position is stored on object in mm

        return positions as np.array in mm
               order is [x, y, z]
        """
        cmd_getposition = [Spi.Commands.position] + Spi.word_bytes * [0]

        for motor in range(self.cfg.hdl_cfg.motors):
            read_data = await self.send_command(cmd_getposition)
            self._position[motor] = unpack("!q", read_data[1:])[0]
            # code below is not portable between python and micropython
            # python requires signed=True, micropython does not accep this
            # overflow error can be generated, if you go below 0
            # overflow is created during the division,
            # it's assumed position cannot be negative.
            # self._position[i] = int.from_bytes(read_data[1:9], 'big', True)
        # step --> mm
        cfg = self.cfg.motor_cfg
        self._position = self._position / np.array(list(cfg["steps_mm"].values()))
        return self._position

    async def send_command(self, command, blocking=False):
        """writes command to spi port

        blocking  --  try again if memory is full
        returns bytearray with length equal to data sent
        """
        assert len(command) == Spi.word_bytes + Spi.command_bytes

        command = bytearray(command)
        response = bytearray(command)

        trials = 0
        while True:
            trials += 1
            if self.test:
                # function is created in Amaranth HDL
                response[:] = await self.spi_exchange_data(command)
            else:
                self.fpga_select.value(0)
                self.spi.write_readinto(command, response)
                self.fpga_select.value(1)
            if not blocking:
                break

            byte1 = response[-1]  # can't rely on self.get_state (needs speed!)
            if (byte1 >> (7 - Spi.State.error)) & 1:
                if not self.test:
                    # lower SPI speed, change FPGA implementation
                    # you cannot recover from this
                    self.reset()
                raise Exception("Error detected on FPGA")
            if not ((byte1 >> (7 - Spi.State.full)) & 1):
                break
            if trials > self.spi_tries:
                raise Memfull(f"Too many trials {trials} needed")

        return response

    def bytetocmdlist(self, bytelst):
        cmdlist = []
        write_byte = Spi.Commands.write.to_bytes(1, "big")
        bytelst_len = len(bytelst)
        for i in range(0, bytelst_len, 8):
            lst = bytelst[i : i + 8]
            lst.reverse()
            data = write_byte + bytes(lst)
            cmdlist.append(data)
        return cmdlist

    def bittobytelist(self, bitlst, stepsperline=1, direction=0):
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
        bitorder = "little"
        bits_in_scanline = self.cfg.laser_timing["bits_in_scanline"]
        halfperiod = int((bits_in_scanline - 1) // (stepsperline * 2))
        if halfperiod < 1:
            raise Exception("Steps per line cannot be achieved")
        # TODO: is this still an issue?
        # you could check as follows
        # steps = self.laser_params['TICKSINFACET']/(halfperiod*2)
        #    print(f"{steps} is actual steps per line")
        direction_byte = [int(bool(direction))]

        def pad_with_zeros(bytelst):
            word_length = Spi.word_bytes
            padding_length = (word_length - len(bytelst) % word_length) % word_length
            bytelst.extend([0] * padding_length)

        if not len(bitlst):
            bytelst = [Spi.Instructions.last_scanline]
            pad_with_zeros(bytelst)
            return bytelst

        bytelst = [Spi.Instructions.scanline]
        halfperiodbits = [int(i) for i in bin(halfperiod)[2:]]
        halfperiodbits.reverse()
        assert len(halfperiodbits) < 56
        bytelst.extend(
            list(ulabext.packbits(direction_byte + halfperiodbits, bitorder=bitorder))
        )
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

    async def writeline(self, bitlst, stepsperline=1, direction=0, repetitions=1):
        """
        Projects the given bit list as a line using the laser head.

        Parameters:
            bitlst (List[int]): Bits to be written to the substrate.
                                Currently, the laser can only be on or off.
                                If empty, a stop command is sent.
            stepsperline (int): Number of motor steps per line.
                                Must be greater than 0. To disable motion,
                                turn off the motor separately.
            direction (int): Direction of laser head movement (0 = backward, 1 = forward).
            repetitions (int): Number of times the line is projected.

        Behavior:
            Converts the bit list into a sequence of commands and
            sends them to the FPGA controller. Commands are repeated
            for the specified number of repetitions.
        """
        bytelst = self.bittobytelist(bitlst, stepsperline, direction)
        for _ in range(repetitions):
            cmdlst = self.bytetocmdlist(bytelst)
            for cmd in cmdlst:
                (await self.send_command(cmd, blocking=True))

    async def enable_comp(
        self,
        laser0=False,
        laser1=False,
        polygon=False,
        synchronize=False,
        singlefacet=False,
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
            [Spi.Commands.write]
            + [0] * (Spi.word_bytes - 2)
            + [int(f"{singlefacet}{synchronize}{polygon}{laser1}{laser0}", 2)]
            + [Spi.Instructions.write_pin]
        )
        await self.send_command(data, blocking=True)

    async def spline_move(self, ticks, coefficients):
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
        cfg = self.cfg.hdl_cfg
        # maximum allowable ticks is move ticks,
        # otherwise counters overflow in FPGA
        assert ticks <= cfg.move_ticks
        assert len(coefficients) % cfg.motors == 0
        write_byte = Spi.Commands.write.to_bytes(1, "big")
        move_byte = Spi.Instructions.move.to_bytes(1, "big")
        commands = [write_byte + ticks.to_bytes(7, "big") + move_byte]
        # check max order given by caller of function
        max_coeff_order = len(coefficients) // cfg.motors
        # prepare commands
        for motor in range(cfg.motors):
            for degree in range(cfg.poldegree):
                # set to zero if coeff not provided by caller
                if degree > max_coeff_order - 1:
                    coeff = 0
                else:
                    idx = degree + motor * max_coeff_order
                    coeff = coefficients[idx]
                if sys.implementation.name == "micropython":
                    data = int(coeff).to_bytes(8, "big", True)
                else:
                    data = int(coeff).to_bytes(8, "big", signed=True)
                commands += [write_byte + data]
        # send commands to FPGA
        for command in commands:
            data_out = await self.send_command(command, blocking=True)
            state = await self.get_state(data_out)
        axes_names = list(self.cfg.motor_cfg["steps_mm"].keys())
        return np.array([state[key] for key in axes_names])

    async def get_state(self, data=None):
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
            command = [Spi.Commands.read] + Spi.word_bytes * [0]
            data = await self.send_command(command)

        dct = {}
        # 9 bytes are returned
        # the state is decoded from byte 7 and 8, i.e. -2 and -1
        byte1 = data[-1]  # Last byte

        dct["parsing"] = (byte1 >> (7 - Spi.State.parsing)) & 1
        dct["error"] = (byte1 >> (7 - Spi.State.error)) & 1
        dct["mem_full"] = (byte1 >> (7 - Spi.State.full)) & 1

        byte2 = data[-2]  # Second to last byte
        mapping = list(self.cfg.motor_cfg["steps_mm"].keys())

        motors = self.cfg.hdl_cfg.motors
        for i in range(motors):
            dct[mapping[i]] = (byte2 >> (7 - i)) & 1
        dct["photodiode_trigger"] = (byte2 >> (7 - motors)) & 1
        dct["synchronized"] = (byte2 >> (7 - motors - 1)) & 1
        return dct

    async def set_parsing(self, value):
        """enables or disables parsing of FIFO by FPGA

        val -- True   FPGA parses FIFO
               False  FPGA does not parse FIFO
        """
        assert isinstance(value, bool)
        if value:
            command = [Spi.Commands.start]
        else:
            command = [Spi.Commands.stop]
        command += Spi.word_bytes * [0]
        return await self.send_command(command)


class TestHost(BaseHost):
    def __init__(self):
        super().__init__(test=True)
        self.spi_tries = 1
        self.fifo_tries = 10
        self._position = np.array([0] * self.cfg.hdl_cfg.motors, dtype=float)

    def build(self, do_program=False, verbose=True, mod="all"):
        """
        Builds the FPGA code using Amaranth HDL, Yosys, Nextpnr, and Icepack.

        Parameters:
        - do_program (bool): If True, flashes the FPGA using fomu-flash and resets afterwards.
        - verbose (bool): If True, prints output of Yosys, Nextpnr, and Icepack.
        - mod (str): Specifies which module to build. Options: 'all', 'motor'.
        """
        platform = Firestarter()
        if mod == "all":
            module = Dispatcher(platform)
        elif mod == "motor":
            module = Driver(platform, top=True)
        else:
            raise Exception(f"Print building {mod} is not supported.")
        platform.build(
            module,
            do_program=do_program,
            verbose=verbose,
        )


class MicropythonHost(BaseHost):
    """
    Host interface to interact with the FPGA.

    Args:
        test (bool): If True, runs in test mode with virtual FPGA platform.

    In test mode, the object uses mock settings and disables MicroPython-specific code.
    """

    def __init__(self, test=False):
        super().__init__(test=False)
        self.steppers_init = False
        self.spi_tries = 3
        self.fifo_tries = 1e5
        self._position = np.array([0] * self.cfg.hdl_cfg.motors, dtype=np.float)
        self.init_micropython()

    def init_micropython(self):
        """Initialize hardware peripherals in a MicroPython environment.

        Sets up GPIO pins, SPI, I2C, and chip selects required for communication
        with the FPGA, flash, and stepper drivers. Also triggers init_steppers.
        """
        cfg = self.cfg.esp32_cfg

        self.fpga_reset = Pin(cfg["fpga_reset"], Pin.OUT)
        self.fpga_reset.value(1)
        self.i2c = I2C(scl=cfg["i2c"]["scl"], sda=cfg["i2c"]["sda"])
        # hardware SPI works partly, set speed to 3e6
        # return bytes give issue in retrieving position
        spi = cfg["spi"]
        self.spi = SPI(
            2,
            baudrate=spi["baudrate"],
            polarity=spi["polarity"],
            phase=spi["phase"],
            sck=Pin(spi["sck"], Pin.OUT),
            mosi=Pin(spi["mosi"], Pin.OUT),
            miso=Pin(spi["miso"], Pin.IN),
        )
        # keep for hardware SPI
        self.spi.deinit()
        self.spi.init()
        self.flash_cs = Pin(cfg["flash_cs"], Pin.OUT)
        self.flash_cs.value(1)
        self.fpga_cs = Pin(cfg["fpga_cs"], Pin.OUT)
        self.stepper_cs = Pin(cfg["stepper_cs"], Pin.OUT)
        self.init_steppers()

    def init_steppers(self, current=100):
        """Configure TMC2209 stepper drivers over UART.

        Sets current, interpolation, and microstepping parameters for each stepper motor.
        Logs failures if drivers are not detected. Only runs once per session.

        Args:
            current (int): Desired motor current in mA (default: 100).
        """
        cfg = self.cfg.esp32_cfg

        if not self.steppers_init:
            failed = False
            for key, value in cfg["tmc2209_uart_ids"].items():
                try:
                    tmc = TMC_2209(pin_en=cfg["stepper_cs"], mtr_id=value)
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
            self.steppers_init = not failed

    async def flash_fpga(self, filename):
        """Flash a bitstream file to the FPGA using SoftSPI and W25QFlash.

        Resets the FPGA, writes the file to flash memory in blocks, and resets again
        after completion.

        Args:
            filename (str): Path to the bitstream file (e.g. .bin) to be flashed.
        """
        cfg = self.cfg.esp32_cfg
        self.fpga_reset.value(0)
        self.fpga_cs.value(1)
        await sleep(1)
        # can't get hardware spi working with memory
        spi = SoftSPI(
            polarity=cfg["spi"]["polarity"],
            phase=cfg["spi"]["phase"],
            sck=Pin(cfg["spi"]["sck"], Pin.OUT),
            mosi=Pin(cfg["spi"]["mosi"], Pin.OUT),
            miso=Pin(cfg["spi"]["miso"], Pin.IN),
        )
        spi.deinit()
        spi.init()

        f = W25QFlash(
            spi=spi,
            cs=cfg["fpga_cs"],
            baud=cfg["spi"]["baudrate"],
            software_reset=True,
        )

        with open(filename, "rb") as infile:
            buffsize = f.BLOCK_SIZE
            blocknum = 0
            while True:
                buf = infile.read(buffsize)
                logging.info(f"Writing block {blocknum}.")
                f.writeblocks(blocknum, buf)
                if len(buf) < buffsize:
                    logging.info(f"Final block {blocknum}")
                    break
                blocknum += 1
        self.reset()
        logging.info("Flashed fpga.")

    async def reset(self):
        "restart the FPGA by toggling the reset pin and initializing communication"
        # free all lines
        self.spi.deinit()
        self.flash_cs.mode(Pin.IN)

        self.fpga_reset.value(0)
        await sleep(1)
        self.fpga_reset.value(1)
        await sleep(1)
        self.init_micropython()
        length = Spi.word_bytes + Spi.command_bytes
        command = bytearray(length)
        await self.send_command(command, blocking=False)

    # async def get_motordebug(self, blocking=False):
    #     """retrieves the motor debug word

    #     This is used to debug the PI controller and
    #     set the correct setting for the Hall interpolation

    #     blocking   -- checks if memory is full, only needed for
    #                   a build with all modules
    #     """
    #     command = [COMMANDS.DEBUG] + WORD_BYTES * [0]
    #     response = (await self.send_command(command, blocking=blocking))[1:]

    #     clock = int(self.platform.clks[self.platform.hfosc_div] * 1e6)
    #     mode = self.platform.laser_timing["MOTORDEBUG"]

    #     def cntcnv(cnt):
    #         if cnt != 0:
    #             speed = (
    #                 clock / (cnt * 4 * self.platform.laser_timing["MOTORDIVIDER"]) * 60
    #             )
    #         else:
    #             speed = 0
    #         return speed

    #     if (mode == "cycletime") & (response != 0):
    #         response = int.from_bytes(response, "big")
    #         # you measure 180 degrees
    #         if response != 0:
    #             response = round((clock / (response * 2) * 60))
    #     elif mode == "PIcontrol":
    #         cnt = int.from_bytes(response[(WORD_BYTES - 2) :], "big", signed=False)
    #         speed = cntcnv(cnt)
    #         duty = int.from_bytes(response[: (WORD_BYTES - 2)], "big", signed=True)
    #         response = [speed, duty]
    #     elif mode == "ticksinfacet":
    #         cnt = int.from_bytes(response[(WORD_BYTES - 2) :], "big", signed=False)
    #         speed = cntcnv(cnt)
    #         cntdiode = int.from_bytes(response[: (WORD_BYTES - 2)], "big", signed=False)
    #         speedd = cntcnv(cntdiode)
    #         response = [speed, speedd]
    #     else:
    #         response = int.from_bytes(response, "big")

    #     if not isinstance(response, list):
    #         return [response]
    #     else:
    #         return response

    @property
    def enable_steppers(self) -> bool:
        """
        Returns whether the stepper motors are enabled.

        Notes:
            - The enable pin for the stepper drivers is not routed via the FPGA.
            - The pin is active-low: LOW (0) = enabled, HIGH (1) = disabled.
            - Even if the motors are enabled, they won't move unless the FPGA
            is parsing instructions from the FIFO buffer.

        Returns:
            bool: True if steppers are enabled, False otherwise.
        """
        return self.stepper_cs.value == 0

    @enable_steppers.setter
    def enable_steppers(self, val: bool):
        """
        Enables or disables the stepper motor drivers.

        Parameters:
        val (bool): True to enable steppers (active-low), False to disable.
        """
        if not isinstance(val, bool):
            raise ValueError("enable_steppers must be a boolean value (True or False)")

        # Assuming 'enable' is active-low: 0 = enabled, 1 = disabled
        self.stepper_cs.value(0 if val else 1)

    @property
    def laser_current(self):
        """
        Returns the laser current as an integer (0–255).

        Notes:
        - Both channels share the same current setting.
        - 0 represents no current; 255 represents full driver current.
        """
        return list(self.bus.readfrom_mem(self.platform.ic_address, 0, 1))[0]

    @laser_current.setter
    def laser_current(self, val):
        """
        Set the maximum laser current for the driver per channel.

        Note:
            - This does not turn the laser on or off.
            - Current is applied when the laser is pulsed.
            - Laser driver accepts values from 0 to 255.
            - Exceeding MAX_SAFE_CURRENT, i.e. 150, may damage the laser.

        Args:
            val (int): Desired laser current value (0–150).

        Raises:
            ValueError: If val is outside the allowed range.
        """
        MAX_SAFE_CURRENT = 150  # Do not exceed this; risks hardware damage

        if not (0 <= val <= MAX_SAFE_CURRENT):
            raise ValueError(
                f"Laser current must be between 0 and {MAX_SAFE_CURRENT} (inclusive)"
            )

        self.bus.writeto_mem(self.platform.ic_address, 0, bytes([val]))

    # async def home_axes(self, axes, speed=None, displacement=-200):
    #     """home given axes, i.e. [1,0,1] homes x, z and not y

    #     axes         -- list with axes to home
    #     speed        -- speed in mm/s used to home
    #     displacement -- displacement used to touch home switch
    #     """
    #     assert len(axes) == self.platform.motors
    #     dist = np.array(axes) * np.array([displacement] * self.platform.motors)
    #     await self.gotopoint(position=dist.tolist(), speed=speed, absolute=False)

    # # TODO: this is strange, should it be here
    # #       on the board steps and count is stored
    # #       you could move this to spline_coefficients
    # #       the flow over method of a certain bit comes from beagleg
    # def steps_to_count(self, steps):
    #     """compute count for a given number of steps

    #     steps  -- motor moves in small steps

    #     Shift is needed as two ticks per step are required
    #     You need to count slightly over the threshold. That is why
    #     +1 is added.
    #     """
    #     bitshift = bit_shift(self.platform)
    #     count = (steps << (1 + bitshift)) + (1 << (bitshift - 1))
    #     return count

    # async def gotopoint(self, position, speed=None, absolute=True):
    #     """move machine to position or with displacement at constant speed

    #     Axes are moved independently to simplify the calculation.
    #     The move is carried out as a first order spline, i.e. only velocity.

    #     position     -- list with position or displacement in mm for each motor
    #     speed        -- list with speed in mm/s, if None default speeds used
    #     absolute     -- True if position, False if displacement
    #     """
    #     (await self.set_parsing(True))
    #     assert len(position) == self.platform.motors
    #     if speed is not None:
    #         assert len(speed) == self.platform.motors
    #     else:
    #         speed = [10] * self.platform.motors
    #     # conversions to steps / count gives rounding errors
    #     # minimized by setting speed to integer
    #     speed = abs(np.array(speed))
    #     displacement = np.array(position)
    #     if absolute:
    #         # TODO: position machine should be in line with self._position
    #         #       which to pick?
    #         displacement -= self._position

    #     homeswitches_hit = [0] * len(position)
    #     for idx, disp in enumerate(displacement):
    #         if disp == 0:
    #             # no displacement, go to next axis
    #             continue
    #         # Time needed for move
    #         #    unit oscillator ticks (times motor position is updated)
    #         time = abs(disp / speed[idx])
    #         ticks_total = round(time * MOTORFREQ)
    #         # mm -> steps
    #         steps_per_mm = list(self.platform.stepspermm.values())[idx]
    #         speed_steps = int(round(speed[idx] * steps_per_mm * ulabext.sign(disp)))
    #         velocity = [0] * len(speed)
    #         velocity[idx] = self.steps_to_count(speed_steps) // MOTORFREQ
    #         (await self.set_parsing(True))

    #         while ticks_total > 0:
    #             ticks_move = MOVE_TICKS if ticks_total >= MOVE_TICKS else ticks_total
    #             # execute move and retrieve if switch is hit
    #             switches_hit = await self.spline_move(int(ticks_move), velocity)
    #             ticks_total -= ticks_move
    #             # move is aborted if home switch is hit and
    #             # velocity is negative
    #             cond = (switches_hit[idx] == 1) & (ulabext.sign(disp) < 0)
    #             if cond:
    #                 break
    #     # update internally stored position
    #     self._position += displacement
    #     # set position to zero if home switch hit
    #     self._position[homeswitches_hit == 1] = 0
    #     # TODO: you enable parsing but don't disable it
    #     #       this would require a wait or maybe it should be enabled on
