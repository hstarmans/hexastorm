import sys
import logging
from asyncio import sleep

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
        self._position = np.array(
            [0] * self.cfg.hdl_cfg.motors, dtype=NP_FLOAT
        )  # machine position

        self._work_offset = np.array([0] * self.cfg.hdl_cfg.motors, dtype=NP_FLOAT)

        # Track the state of the 8-bit 'write_pin' register locally
        # Bits 0-4: laser0, laser1, polygon, synchronize, singlefacet
        # Bits 5-7: led0, led1, led2
        self._pin_state = 0

        self._fan_speed = 0
        self._spindle_speed = 0

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

    async def _send_pin_state(self):
        """Helper to send the current internal pin state to the FPGA."""
        data = (
            [Spi.Commands.write]
            + [0] * (Spi.word_bytes - 2)
            + [self._pin_state]
            + [Spi.Instructions.write_pin]
        )
        await self.send_command(data)

    async def set_leds(self, blue=None, green=None, red=None):
        """
        Turn the debugging LEDs on or off.

        Args:
            blue (bool): State of the blue LED. If None, remains unchanged.
            green (bool): State of the green LED. If None, remains unchanged.
            red (bool): State of the red LED. If None, remains unchanged.
        """
        if blue is not None:
            if blue:
                self._pin_state |= 1 << 5
            else:
                self._pin_state &= ~(1 << 5)

        if green is not None:
            if green:
                self._pin_state |= 1 << 6
            else:
                self._pin_state &= ~(1 << 6)

        if red is not None:
            if red:
                self._pin_state |= 1 << 7
            else:
                self._pin_state &= ~(1 << 7)

        await self._send_pin_state()

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
        # Clear the lower 5 bits (hardware components), but keep the upper 3 bits (LEDs) intact
        self._pin_state &= 0xE0  # 0xE0 is 11100000 in binary

        flags = (
            (int(singlefacet) << 4)
            | (int(synchronize) << 3)
            | (int(polygon) << 2)
            | (int(laser1) << 1)
            | int(laser0)
        )

        # Apply the new hardware flags
        self._pin_state |= flags
        await self._send_pin_state()

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

    async def read_switches(self):
        """
        Read the state of the home switches for all axes.

        Returns:
            np.ndarray: Boolean array indicating the state of each axis's home switch.
        """
        state = await self._read_fpga_state()
        axis_names = list(self.cfg.motor_cfg["steps_mm"].keys())
        return np.array([state[key] for key in axis_names])

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

    async def flush_buffer(self):
        """
        Immediately clears the FPGA instruction FIFO buffer.

        This sends a dedicated SPI command that triggers a hardware-level
        pointer reset, instantly emptying the queue of pending instructions.
        """
        command = [Spi.Commands.flush] + [0] * Spi.word_bytes
        return await self.send_command(command)

    async def home_axes(self, axes, speed=None, displacement=200, pull_off=10):
        """Home given axes, i.e. [1,0,1] homes x, z and not y.

        Args:
        axes         -- list with axes to home (e.g., [1, 0, 1])
        speed        -- speed in mm/s used to home
        displacement -- displacement used to touch home switch
        pull_off     -- displacement in mm to back off after hitting the switch
        """
        mtrs = self.cfg.hdl_cfg.motors
        assert len(axes) == mtrs

        axis_names = list(self.cfg.motor_cfg["steps_mm"].keys())

        # 1. Quickly extract homing directions using a list comprehension
        # Defaults to -1 if the axis or 'homing_dir' isn't explicitly defined
        homing_dirs = [
            self.cfg.motor_cfg.get("homing_dir", -1)[ax] for ax in axis_names
        ]

        # 2. Use numpy/ulab for vectorized displacement calculation
        # e.g., [1, 0, 1] * [-1, -1, 1] * 200 = [-200, 0, 200]
        dist = np.array(axes) * np.array(homing_dirs) * abs(displacement)

        # Execute the move to the home switches
        hit_array = await self.gotopoint(
            position=dist.tolist(), speed=speed, absolute=False, check_sensors=True
        )
        valid_hits = np.array(axes) * np.array(hit_array)
        multiplier = 1 - valid_hits
        self._position = self._position * multiplier

        if self.cfg.test:
            # skipping back-off after homing
            return
        else:
            # 3. Pull-off in the EXACT OPPOSITE direction of the homing move
            # We multiply by -abs(pull_off) to invert the vector
            # e.g., [1, 0, 1] * [-1, -1, 1] * -10 = [10, 0, -10]
            offset_mm = np.array(
                [self.cfg.motor_cfg.get("offset_mm", -1)[ax] for ax in axis_names]
            )
            back_off_dist = np.array(axes) * offset_mm

            await self.gotopoint(
                position=back_off_dist.tolist(),
                speed=speed,
                absolute=False,
                check_sensors=False,
            )

    async def gotopoint(
        self,
        position,
        speed=None,
        absolute=True,
        check_sensors=True,
        workspace=False,
        blind_distance_mm=5.0,
    ):
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
        check_sensors -- If True, aborts motion if switch threshold is passed (home switch hit).
        workspace    -- If True, treats absolute targets as workspace coordinates (WPOS)
                        rather than machine coordinates (MPOS). This has no effect if 'absolute' is False. Defaults to False.
        blind_distance_mm -- Distance in mm at the start of the move to ignore sensor triggers (overcomes start-up spikes).
        """
        await self.set_parsing(True)
        hdl_cfg = self.cfg.hdl_cfg
        num_axes = hdl_cfg.motors
        steps_per_mm = list(self.cfg.motor_cfg["steps_mm"].values())

        # Validation
        assert len(position) == num_axes
        if speed is None:
            speed = [10] * num_axes
        else:
            assert len(speed) == num_axes

        # Precompute
        position = np.array(position)
        speed = abs(np.array(speed))

        if absolute:
            if workspace:
                # Target is in workspace coordinates, translate to absolute machine coordinates
                target_mpos = position + self._work_offset
                displacement = target_mpos - self._position
            else:
                # Target is direct absolute machine coordinates
                displacement = position - self._position
        else:
            # Relative movement (JOG) is unaffected by coordinate system selection
            displacement = position

        homeswitches_hit = [0] * num_axes

        for axis, disp_mm in enumerate(displacement):
            if disp_mm == 0:
                continue

            # Calculate total duration and ticks
            axis_name = list(self.cfg.motor_cfg["steps_mm"].keys())[axis]
            duration_s = abs(disp_mm / speed[axis])
            total_ticks = int(round(duration_s * hdl_cfg.motor_freq))
            ticks_remaining = total_ticks

            # Calculate how many ticks represent the blind distance
            # Ensure blind ticks don't exceed total ticks
            blind_duration_s = min(abs(blind_distance_mm / speed[axis]), duration_s)
            blind_ticks = int(round(blind_duration_s * hdl_cfg.motor_freq))

            speed_steps = int(
                round(speed[axis] * steps_per_mm[axis] * ulabext.sign(disp_mm))
            )
            velocity = [0] * num_axes
            velocity[axis] = self.steps_to_count(speed_steps) // hdl_cfg.motor_freq

            while ticks_remaining > 0:
                ticks_chunk = min(ticks_remaining, hdl_cfg.move_ticks)
                switches = await self.spline_move(int(ticks_chunk), velocity)
                ticks_remaining -= ticks_chunk

                # Calculate how many ticks we have processed so far
                ticks_processed = total_ticks - ticks_remaining

                # Check sensors only if we are past the blind distance
                if check_sensors and (ticks_processed > blind_ticks):
                    if switches[axis]:
                        if self.test:
                            homeswitches_hit[axis] = 1
                            break
                        else:
                            extra_samples = 4
                            hits_count = 1

                            for _ in range(extra_samples):
                                await sleep(0.010)  # Wait 10 milliseconds
                                new_switches = await self.read_switches()
                                if new_switches[axis]:
                                    hits_count += 1

                            if hits_count >= (extra_samples // 2) + 1:
                                await self.set_parsing(
                                    False
                                )  # Stop parsing new commands
                                await (
                                    self.flush_buffer()
                                )  # Obliterate the pending buffer

                                state = self.enable_steppers
                                self.enable_steppers = False
                                await sleep(
                                    0.5
                                )  # Let physics settle (buffer is already cleared)
                                self.enable_steppers = state

                                homeswitches_hit[axis] = 1
                                logger.warning(
                                    f"Home switch hit on axis {axis_name} during move. Aborting motion."
                                )
                                break
                            else:
                                # false trigger, continue moving
                                pass

        # Note: If a move is aborted via flush, self._position might assume the full
        # displacement occurred. You may need to query actual position if absolute tracking is required after an abort.
        self._position += displacement

        return homeswitches_hit

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

    @property
    def fan_speed(self):
        """
        Returns the last set fan speed (0-255).

        This value is retrieved from the local host cache to minimize SPI overhead.
        """
        return self._fan_speed

    async def set_fan_speed(self, speed: int):
        """
        Sets the cooling fan PWM duty cycle.

        Args:
            speed (int): Target speed / duty cycle value from 0 (Off) to 255 (100% On).
        """
        self._fan_speed = max(0, min(255, int(speed)))

        # Structure payload: Opcode inside write command block
        # Byte layout expects: Write command + Padding + Data payload + Instruction Opcode
        data = (
            [Spi.Commands.write]
            + [0] * (Spi.word_bytes - 2)
            + [self._fan_speed]
            + [Spi.Instructions.set_fan]
        )
        await self.send_command(data)

    @property
    def spindle_speed(self):
        """
        Returns the last set spindle speed (0-255).

        This value is retrieved from the local host cache to minimize SPI overhead.
        """
        return self._spindle_speed

    async def set_spindle_speed(self, speed: int):
        """
        Sets the main spindle motor PWM duty cycle.

        Args:
            speed (int): Target speed / duty cycle value from 0 (Off) to 255 (100% On).
        """
        self._spindle_speed = max(0, min(255, int(speed)))

        # Structure payload matching standard instruction layout
        data = (
            [Spi.Commands.write]
            + [0] * (Spi.word_bytes - 2)
            + [self._spindle_speed]
            + [Spi.Instructions.set_spindle]
        )
        await self.send_command(data)

    @property
    def mpos(self):
        """
        Get the absolute machine position (MPOS) as a standard Python list.

        Returns:
            list[float]: The absolute machine coordinates for all motor axes.
        """
        return self._position.tolist()

    @property
    def wpos(self):
        """
        Get the current workspace position (WPOS) as a standard Python list.

        This calculates the position relative to the established workspace zero point
        (Machine Position minus Workspace Offset).

        Returns:
            list[float]: The workspace coordinates for all motor axes.
        """
        return (self._position - self._work_offset).tolist()

    def set_workspace_zero(self, axes=None):
        """
        Set the workspace zero point (WPOS = 0) at the current machine position.

        This updates the internal work offsets, aligning the workspace coordinate system
        to zero for the specified axes.

        Args:
            axes (list[int], optional): A binary mask list (e.g., [1, 1, 0]) indicating which axes
                to zero. If None, all available axes will be zeroed.
        """
        if axes is None:
            axes = [1] * self.cfg.hdl_cfg.motors

        mask = np.array(axes)
        # Retain the old offset where mask is 0, update with current mpos where mask is 1
        self._work_offset = (self._work_offset * (1 - mask)) + (self._position * mask)
