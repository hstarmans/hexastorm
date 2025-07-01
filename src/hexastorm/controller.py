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
        """
        Retrieve the current stepper motor positions from the FPGA and update internal state.

        - FPGA stores position in steps (signed 64-bit integers).
        - Positions are converted to millimeters using steps/mm config.
        - Internal `_position` is updated and returned as a NumPy array in mm.

        Returns:
            np.ndarray: Current motor positions in mm, ordered [x, y, z].
        """
        cmd = [Spi.Commands.position] + [0] * Spi.word_bytes
        num_motors = self.cfg.hdl_cfg.motors

        for motor in range(num_motors):
            read_data = await self.send_command(cmd)
            self._position[motor] = unpack("!q", read_data[1:])[0]
            # code below is not portable between python and micropython
            # python requires signed=True, micropython does not accep this
            # overflow error can be generated, if you go below 0
            # overflow is created during the division,
            # it's assumed position cannot be negative.
            # self._position[i] = int.from_bytes(read_data[1:9], 'big', True)
        # Convert steps to mm
        steps_per_mm = np.array(list(self.cfg.motor_cfg["steps_mm"].values()))
        self._position = self._position / steps_per_mm
        return self._position

    async def send_command(self, command, blocking=False):
        """
        Send a command to the FPGA via SPI and return the response.

        Args:
            command (list[int] or bytearray): Full command consisting of command byte + data bytes.
            blocking (bool): If True, retry sending if the FPGA's memory is full.

        Returns:
            bytearray: Response from the FPGA, same length as the input command.

        Raises:
            Memfull: If too many retries are needed due to a full FIFO.
            Exception: If an error is reported by the FPGA.
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

            status_byte = response[-1]  # can't rely on self.get_state (needs speed!)
            if self._bitflag(status_byte, Spi.State.error):
                if not self.test:
                    self.reset()  # SPI speed or protocol mismatch → unrecoverable
                raise Exception("Error detected on FPGA")
            if not self._bitflag(status_byte, Spi.State.full):
                break  # FIFO has space, continue
            if trials > self.spi_tries:
                raise Memfull(f"Too many retries ({trials}) due to full FIFO")

        return response

    def _bitflag(self, byte, index):
        """Return True if the bit at 'index' in 'byte' is set (0 = LSB)."""
        return bool((byte >> index) & 1)

    def byte_to_cmd_list(self, byte_lst):
        """
        Converts a byte list into a list of SPI commands with write instructions.

        Args:
            bytelst (List[int]): List of bytes to send.

        Returns:
            List[bytes]: List of SPI command packets.
        """
        cmd_list = []
        write_byte = Spi.Commands.write.to_bytes(1, "big")

        for i in range(0, len(byte_lst), 8):
            chunk = list(
                reversed(byte_lst[i : i + 8])
            )  # Reverse in place for SPI endian behavior
            cmd_list.append(write_byte + bytes(chunk))

        return cmd_list

    def bit_to_byte_list(self, laser_bits, steps_line=1, direction=0):
        """
        Convert a bit list into a padded byte list suitable for FPGA scanline commands.

        Args:
            laser_bits (List[int]): Bits to write to the substrate (laser on/off).
            steps_line (int): Number of motor steps for the scanline. Must be > 0.
            direction (int): 0 for backward, 1 for forward.

        Returns:
            List[int]: Byte list ready to be packed into SPI commands.
        """
        # the half_period is sent over
        # this is the amount of ticks in half a cycle of
        # the motor
        # watch out for python "banker's rounding"
        # sometimes target might not be equal to steps
        bit_order = "little"
        scanline_length = self.cfg.laser_timing["scanline_length"]
        half_period = int((scanline_length - 1) // (steps_line * 2))
        if half_period < 1:
            raise Exception("Steps per line cannot be achieved")
        # TODO: is this still an issue?
        # you could check as follows
        # steps = self.laser_params['TICKSINFACET']/(halfperiod*2)
        #    print(f"{steps} is actual steps per line")
        direction_byte = [int(bool(direction))]

        def pad_to_word_boundary(byte_lst):
            word_len = Spi.word_bytes
            padding_length = (word_len - len(byte_lst) % word_len) % word_len
            byte_lst.extend([0] * padding_length)

        if not len(laser_bits):
            byte_lst = [Spi.Instructions.last_scanline]
            pad_to_word_boundary(byte_lst)
            return byte_lst

        # First instruction: scanline + encoded halfperiod and direction
        byte_lst = [Spi.Instructions.scanline]

        # Pack direction + halfperiod bits into little-endian bytes
        half_period_bits = [int(i) for i in bin(half_period)[2:]]
        half_period_bits.reverse()
        assert len(half_period_bits) < 56
        byte_lst.extend(
            list(
                ulabext.packbits(direction_byte + half_period_bits, bitorder=bit_order)
            )
        )
        pad_to_word_boundary(byte_lst)

        # Append actual scanline data
        laser_bits_len = len(laser_bits)
        if laser_bits_len == scanline_length:
            byte_lst.extend(ulabext.packbits(laser_bits, bitorder=bit_order))
        elif laser_bits_len == scanline_length // 8:
            byte_lst.extend(laser_bits)
        else:
            raise Exception("Invalid line length")

        pad_to_word_boundary(byte_lst)
        return byte_lst

    async def write_line(self, bit_lst, steps_line=1, direction=0, repetitions=1):
        """
        Projects a scanline to the substrate using the laser system.

        Args:
            bitlst (List[int]): Laser on/off bits. Empty list sends stop command.
            stepsperline (int): Number of steps to move during scanline.
                                To disable motion, turn off the motor separately.
            direction (int): 0 = backward, 1 = forward.
            repetitions (int): Number of times to repeat the scanline projection.

        Behavior:
            Converts the bit list into a sequence of commands and
            sends them to the FPGA controller. Commands are repeated
            for the specified number of repetitions.
        """
        byte_lst = self.bit_to_byte_list(bit_lst, steps_line, direction)
        for _ in range(repetitions):
            cmd_lst = self.byte_to_cmd_list(byte_lst)
            for cmd in cmd_lst:
                (await self.send_command(cmd, blocking=True))

    async def enable_comp(
        self,
        laser0=False,
        laser1=False,
        polygon=False,
        synchronize=False,
        singlefacet=False,
    ):
        """
        Enable or disable hardware components via a direct SPI instruction.

        Note:
            The FPGA must be parsing FIFO for this to take effect.

        Args:
            laser0 (bool): Enable laser channel 0.
            laser1 (bool): Enable laser channel 1.
            polygon (bool): Enable polygon motor (True = enable).
            synchronize (bool): Enable synchronization feature.
            singlefacet (bool): Enable single-facet mode.
        """
        flags = (
            (int(singlefacet) << 4)
            | (int(synchronize) << 3)
            | (int(polygon) << 2)
            | (int(laser1) << 1)
            | int(laser0)
        )

        data = (
            [Spi.Commands.write]
            + [0] * (Spi.word_bytes - 2)
            + [flags]
            + [Spi.Instructions.write_pin]
        )
        await self.send_command(data, blocking=True)

    # TODO: this is strange, should it be here
    #       on the board steps and count is stored
    #       you could move this to spline_coefficients
    #       the flow over method of a certain bit comes from beagleg
    def steps_to_count(self, steps):
        """Convert a number of motor steps to the corresponding count value.

        Each step requires two ticks. The final count includes a small adjustment
        to ensure the threshold is slightly exceeded.

        Parameters:
        steps (int): Number of small motor steps

        Returns:
        int: The count value for the given number of steps.
        """
        bit_shift = self.cfg.hdl_cfg.bit_shift
        count = (steps << (1 + bit_shift)) + (1 << (bit_shift - 1))
        return count

    async def spline_move(self, ticks, coefficients):
        """
        Send a spline move instruction to the FPGA FIFO.

        This function writes a time-based polynomial trajectory (spline) move for each motor.
        The instruction consists of a tick count and a set of coefficients per motor axis.

        Args:
            ticks (int): Duration of the move in clock ticks. Must be ≤ max allowed by FPGA.
            coefficients (list[int]): Flattened list of coefficients, grouped by axis.
                For example: [x0, x1, x2, y0, y1, y2] for a 2-axis, 3rd-order spline.

        Behavior:
            - If fewer coefficients are provided than the FPGA's configured spline order,
              missing coefficients are padded with zeros.
            - This function assumes coefficients are grouped by axis, not by degree.

        Returns:
            np.ndarray: Boolean array per axis (e.g., [0, 1]) indicating home switch status after move.
        """
        cfg = self.cfg.hdl_cfg
        max_order = cfg.pol_degree
        num_motors = cfg.motors

        # Validate input
        assert ticks <= cfg.move_ticks
        assert len(coefficients) % num_motors == 0

        coeffs_per_motor = len(coefficients) // num_motors

        # Initial move command: ticks + move opcode
        write_byte = Spi.Commands.write.to_bytes(1, "big")
        move_byte = Spi.Instructions.move.to_bytes(1, "big")
        command = write_byte + ticks.to_bytes(7, "big") + move_byte
        commands = [command]

        # Add coefficients (pad with zeros if caller gave fewer than FPGA expects)
        for motor in range(num_motors):
            for degree in range(max_order):
                if degree >= coeffs_per_motor:
                    coeff = 0
                else:
                    idx = degree + motor * coeffs_per_motor
                    coeff = coefficients[idx]
                if sys.implementation.name == "micropython":
                    coeff_bytes = int(coeff).to_bytes(8, "big", True)
                else:
                    coeff_bytes = int(coeff).to_bytes(8, "big", signed=True)
                commands += [write_byte + coeff_bytes]

        # Send each command and track final response
        for command in commands:
            data_out = await self.send_command(command, blocking=True)

        # Decode the home switch state after the move
        state = await self._read_fpga_state(data_out)
        axis_names = list(self.cfg.motor_cfg["steps_mm"].keys())
        return np.array([state[key] for key in axis_names])

    @property
    def fpga_state(self):
        """
        Async accessor for the current FPGA state.

        Usage:
            state = await host.fpga_state
        """
        return self._read_fpga_state()

    async def _read_fpga_state(self, data=None):
        """Retrieve the state of the FPGA as a dictionary.

        Args:
            data (Optional[List[int]]): Raw byte data to decode. If None, the data
            will be retrieved from the FPGA via a read command.

        Returns:
        dict: A dictionary with the following keys:
            - parsing (bool): True if commands are being executed.
            - mem_full (bool): True if the FIFO memory is full.
            - error (bool): True if any submodule has entered an error state.
            - x, y, z (bool): State of the motor end switches (names depend on config).
            - photodiode_trigger (bool): True if photodiode was triggered during last prism rotation.
            - synchronized (bool): True if the prism is tracked by the photodiode.
        """
        if data is None:
            command = [Spi.Commands.read] + [0] * Spi.word_bytes
            data = await self.send_command(command)

        # Decode status bits from the last two bytes
        status_byte = data[-1]  # Byte 8 (index -1)
        pin_byte = data[-2]  # Byte 7 (index -2)

        state = {
            "parsing": self._bitflag(status_byte, Spi.State.parsing),
            "error": self._bitflag(status_byte, Spi.State.error),
            "mem_full": self._bitflag(status_byte, Spi.State.full),
        }

        motor_keys = list(self.cfg.motor_cfg["steps_mm"].keys())
        motor_count = self.cfg.hdl_cfg.motors

        for i in range(motor_count):
            motor_name = motor_keys[i] if i < len(motor_keys) else f"motor_{i}"
            state[motor_name] = bool((pin_byte >> i) & 1)

        state["photodiode_trigger"] = self._bitflag(pin_byte, motor_count)
        state["synchronized"] = self._bitflag(pin_byte, motor_count + 1)

        return state

    async def set_parsing(self, enabled):
        """
        Enable or disable FIFO parsing on the FPGA.

        Args:
            enabled (bool):
                True  → FPGA begins parsing instructions from FIFO.
                False → FPGA stops parsing.
        """
        if not isinstance(enabled, bool):
            raise TypeError("set_parsing() expects a boolean")

        cmd = Spi.Commands.start if enabled else Spi.Commands.stop
        command = [cmd] + [0] * Spi.word_bytes
        return await self.send_command(command)

    async def home_axes(self, axes, speed=None, displacement=-200):
        """home given axes, i.e. [1,0,1] homes x, z and not y

        axes         -- list with axes to home
        speed        -- speed in mm/s used to home
        displacement -- displacement used to touch home switch
        """
        mtrs = self.cfg.hdl_cfg.motors
        assert len(axes) == mtrs
        dist = np.array(axes) * np.array([displacement] * mtrs)
        await self.gotopoint(position=dist.tolist(), speed=speed, absolute=False)

    async def gotopoint(self, position, speed=None, absolute=True):
        """move machine to position or with displacement at constant speed

        Axes are moved independently to simplify the calculation.
        The move is carried out as a first order spline, i.e. only velocity.

        position     -- list with position or displacement in mm for each motor
        speed        -- list with speed in mm/s, if None default speeds used
        absolute     -- True if position, False if displacement
        """
        (await self.set_parsing(True))
        hdl_cfg = self.cfg.hdl_cfg
        mtrs = hdl_cfg.motors
        assert len(position) == mtrs
        if speed is not None:
            assert len(speed) == mtrs
        else:
            speed = [10] * mtrs
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
            ticks_total = round(time * hdl_cfg.motor_freq)
            # mm -> steps
            steps_per_mm = list(self.cfg.motor_cfg["steps_mm"].values())[idx]
            speed_steps = int(round(speed[idx] * steps_per_mm * ulabext.sign(disp)))
            velocity = [0] * len(speed)
            velocity[idx] = self.steps_to_count(speed_steps) // hdl_cfg.motor_freq
            (await self.set_parsing(True))

            while ticks_total > 0:
                ticks_move = (
                    hdl_cfg.move_ticks
                    if ticks_total >= hdl_cfg.move_ticks
                    else ticks_total
                )
                # execute move and retrieve if switch is hit
                switches_hit = await self.spline_move(int(ticks_move), velocity)
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


class TestHost(BaseHost):
    def __init__(self):
        super().__init__(test=True)
        self.spi_tries = 10
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
        self.spi_tries = 1e5
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

    @property
    def enable_steppers(self):
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
    def enable_steppers(self, val):
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
