from struct import unpack
import sys
import logging

from .. import ulabext
from ..config import Spi, PlatformConfig

try:
    import numpy as np

    NP_FLOAT = float
except ImportError:
    from ulab import numpy as np

    NP_FLOAT = np.float

logger = logging.getLogger(__name__)


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
        # mpy requires np.float
        self._position = np.array([0] * self.cfg.hdl_cfg.motors, dtype=NP_FLOAT)

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
        steps_per_mm = np.array(list(self.cfg.motor_cfg["steps_mm"].values()))
        for motor in range(num_motors):
            read_data = await self.send_command(cmd)
            # Convert steps to mm
            self._position[motor] = (
                float(unpack("!i", read_data[-4:])[0]) / steps_per_mm[motor]
            )

        return self._position

    async def send_command(self, command, timeout=0):
        """
        Send a command to the FPGA via SPI and return the response.

        Args:
            command (list[int] or bytearray): Full command consisting of command byte + data bytes.
            timeout (boolean): Whether to use a timeout for the command.

        Returns:
            bytearray: Response from the FPGA, same length as the input command.
        """
        pass  # implemented in subclasses

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
        word_len = Spi.word_bytes
        byteorder = "little"
        scanline_length = self.cfg.laser_timing["scanline_length"]
        half_period = int((scanline_length - 1) // (steps_line * 2))
        if half_period < 1:
            raise ValueError("Steps per line cannot be achieved (period < 1)")

        # 2. Build Header using Bitwise Math
        # The FPGA expects 56 bits: [55 bits for Ticks] + [1 bit for Direction]
        # In Little Endian, the Direction is the Least Significant Bit (LSB).
        # So we shift Ticks left by 1, and OR in the Direction.

        payload_int = (half_period << 1) | (direction & 1)

        # Convert integer to 7 bytes (56 bits), little endian
        payload_bytes = payload_int.to_bytes(7, byteorder)

        # Create buffer starting with Instruction Byte
        # We use bytearray for performance (mutable), converting to list at the end only if strictly needed
        out_buffer = bytearray()

        # Handle Empty/Stop Case
        if len(laser_bits) == 0:
            out_buffer.append(Spi.Instructions.last_scanline)
        else:
            out_buffer.append(Spi.Instructions.scanline)
            out_buffer.extend(payload_bytes)

        # 3. Header Alignment (Padding)
        # Calculate how many zeros we need to reach the next word boundary
        # This replaces the inner helper function to save function call overhead
        current_len = len(out_buffer)
        padding = (word_len - (current_len % word_len)) % word_len
        if padding:
            out_buffer.extend(b"\x00" * padding)

        # 4. Append Laser Data
        if len(laser_bits) > 0:
            # Case A: Input is raw bits (0, 1, 1, 0...) -> Pack them
            if len(laser_bits) == scanline_length:
                out_buffer.extend(ulabext.packbits(laser_bits, bitorder=byteorder))
            # Case B: Input is already bytes -> Just append
            elif len(laser_bits) == scanline_length // 8:
                out_buffer.extend(laser_bits)
            else:
                raise ValueError(f"Invalid laser_bits length: {len(laser_bits)}")

        # 5. Final Alignment (Padding)-
        current_len = len(out_buffer)
        padding = (word_len - (current_len % word_len)) % word_len
        if padding:
            out_buffer.extend(b"\x00" * padding)

        return list(out_buffer)

    async def write_line(
        self, bit_lst, steps_line=1, direction=0, repetitions=1, facet=None
    ):
        """
        Projects a scanline to the substrate using the laser system.

        Args:
            bitlst (List[int]): Laser on/off bits. Empty list sends stop command.
            stepsperline (int): Number of steps to move during scanline.
                                To disable motion, turn off the motor separately.
            direction (int): 0 = backward, 1 = forward.
            repetitions (int): Number of times to repeat the scanline projection.
            facet (int): Which facet to target (0 to facets-1).

        Behavior:
            Converts the bit list into a sequence of commands and
            sends them to the FPGA controller. Commands are repeated
            for the specified number of repetitions.
        """
        active_byte_lst = self.bit_to_byte_list(bit_lst, steps_line, direction)
        active_cmd_bytes = b"".join(self.byte_to_cmd_list(active_byte_lst))
        packet_size = self.cfg.hdl_cfg.lines_chunk
        if facet is None:
            cmd_bytes = active_cmd_bytes
        else:
            # Create silent command for other facets
            facets = self.cfg.laser_timing["facets"]
            silent_bits = [0] * len(bit_lst)
            silent_byte_lst = self.bit_to_byte_list(silent_bits, steps_line, direction)
            silent_cmd_bytes = b"".join(self.byte_to_cmd_list(silent_byte_lst))

            # Build full cycle with one active facet
            facet_cycle = [silent_cmd_bytes] * facets
            if 0 <= facet < facets:
                facet_cycle[facet] = active_cmd_bytes
            else:
                raise ValueError(f"facet must be between 0 and {facets - 1}")
            cmd_bytes = b"".join(facet_cycle)
            packet_size //= facets

        for i in range(0, repetitions, packet_size):
            last_index = min(i + packet_size, repetitions)
            repeat = last_index - i
            await self.send_command(cmd_bytes * repeat, timeout=True)

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
        await self.send_command(data)

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
            data_out = await self.send_command(command, timeout=True)

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
        """Home given axes, i.e. [1,0,1] homes x, z and not y.

        Args:
        axes         -- list with axes to home
        speed        -- speed in mm/s used to home
        displacement -- displacement used to touch home switch
        """
        mtrs = self.cfg.hdl_cfg.motors
        assert len(axes) == mtrs
        dist = np.array(axes) * np.array([displacement] * mtrs)
        await self.gotopoint(position=dist.tolist(), speed=speed, absolute=False)

    async def gotopoint(self, position, speed=None, absolute=True):
        """Move machine to position by a displacement at constant speed.

        The motion profile is first-order (constant velocity) and each axis
        is driven independently, simplifying timing calculations.

        Args:
        position     --
            • If *absolute* is True,  absolute position (mm) for every axis.
            • If *absolute* is False, relative displacement (mm) for every axis.
        speed        -- list with speed in mm/s, if None default speeds used
        absolute     -- Position interpreted as absolute (True) or
                        a displacement (False).
        """
        await self.set_parsing(True)
        hdl_cfg = self.cfg.hdl_cfg
        num_axes = hdl_cfg.motors
        steps_per_mm = list(self.cfg.motor_cfg["steps_mm"].values())

        #  validation
        assert len(position) == num_axes
        if speed is None:
            speed = [10] * num_axes
        else:
            assert len(speed) == num_axes

        # precompute
        # conversions to steps / count gives rounding errors
        # minimized by setting speed to integer
        position = np.array(position)
        speed = abs(np.array(speed))
        displacement = position - self._position if absolute else position

        homeswitches_hit = [0] * num_axes
        for axis, disp_mm in enumerate(displacement):
            if disp_mm == 0:
                continue

            duration_s = abs(disp_mm / speed[axis])
            ticks_remaining = int(round(duration_s * hdl_cfg.motor_freq))

            speed_steps = int(
                round(speed[axis] * steps_per_mm[axis] * ulabext.sign(disp_mm))
            )
            velocity = [0] * num_axes
            velocity[axis] = self.steps_to_count(speed_steps) // hdl_cfg.motor_freq

            while ticks_remaining > 0:
                ticks_chunk = min(ticks_remaining, hdl_cfg.move_ticks)
                switches = await self.spline_move(int(ticks_chunk), velocity)
                ticks_remaining -= ticks_chunk
                # abort home switch hit and speed negative
                if switches[axis] and (ulabext.sign(disp_mm) < 0):
                    homeswitches_hit[axis] = 1
                    break

        self._position += displacement
        self._position[homeswitches_hit == 1] = 0
        # parsing not disabled !

    async def read_facet_ticks_and_id(self):
        """
        Retrieves facet number and ticks in period.

        Always returns a list [ticks_per_facet, facet_count].
        """
        word_size = Spi.word_bytes
        command = [Spi.Commands.debug] + [0] * word_size
        raw = await self.send_command(command)

        # strip command echo; normalize to bytes
        payload = bytes(raw[1:])
        # last 1 byte: facet count, first 7 bytes: ticks per facet (both unsigned)
        facet_cnt = int.from_bytes(payload[word_size - 1 :], "big")
        ticks_facet = int.from_bytes(payload[: word_size - 1], "big")
        return [ticks_facet, facet_cnt]
