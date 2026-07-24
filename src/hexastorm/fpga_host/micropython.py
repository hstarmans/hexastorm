from asyncio import sleep, sleep_ms, wait_for
import time
import logging
import os
from random import randint
from machine import Pin, SPI, I2C, PWM

from ulab import numpy as np
from tmc.uart import ConnectionFail
from tmc.stepperdriver import TMC_2209

from .syncwrap import syncable
from .interface import BaseHost
from ..config import Spi

logger = logging.getLogger(__name__)


class ESP32Host(BaseHost):
    """
    Host interface to interact with the FPGA using micropython.
    """

    def __init__(self):
        super().__init__(test=False)
        self.steppers_init = False
        self.reset()

    def init_micropython(self):
        """Initialize hardware peripherals in a MicroPython environment.

        Sets up GPIO pins, SPI, I2C, and chip selects required for communication
        with the FPGA, flash, and stepper drivers.
        """
        cfg = self.cfg.esp32_cfg
        ice40_cfg = self.cfg.ice40_cfg
        if ice40_cfg["hfosc_div"] == "esp32s3":
            self.clock = PWM(
                Pin(cfg["clk"]["pin"], Pin.OUT),
                freq=int(ice40_cfg["clks"][ice40_cfg["hfosc_div"]] * 1e6),
                duty=cfg["clk"]["duty"],
            )
        self.fpga_reset = Pin(cfg["fpga"]["reset"], Pin.OUT)
        self.fpga_reset.value(1)
        # digipot and camera are on the same i2c bus
        # not all frequencies work, 100kHz is the most stable
        self.i2c = I2C(
            scl=cfg["i2c"]["scl"], sda=cfg["i2c"]["sda"], freq=cfg["i2c"]["freq"]
        )
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
        self.fpga_cs = Pin(cfg["fpga"]["cs"], Pin.OUT)
        self.fpga_done = Pin(cfg["fpga"]["done"], Pin.IN)
        self.stepper_cs = Pin(cfg["stepper_cs"], Pin.OUT)
        self._mem_full = Pin(cfg["fpga"]["mem_full"], Pin.IN)

    @property
    def mem_full(self):
        """
        Returns whether the memory buffer is full (boolean).
        """
        return self._mem_full.value()

    async def wait_mem_empty(self):
        """
        Simulates a task waiting for memory to be cleared.
        This will loop indefinitely in this example.
        """
        while self.mem_full:
            await sleep(0)

    async def send_command(self, command, timeout=0, debug=False):
        """
        Send a command to the FPGA via SPI and return the response.

        Args:
            command (list[int] or bytearray): Full command consisting of command byte + data bytes.
            timeout (boolean): Whether to use a timeout for the command.

        Returns:
            bytearray: Response from the FPGA, same length as the input command.

        Raises:
            TimeoutError: If too much time is needed due to a full FIFO.
            Exception: If an error is reported by the FPGA.
        """
        command = bytearray(command)
        response = bytearray(command)

        if timeout and self.mem_full:
            if debug:
                logger.info("Memory full, waiting for FIFO to empty")
                start_time = time.ticks_ms()
            # tried IRQ call back with await for --> cannot get it working
            # the call back never propagates even with micropython.schedule
            # 254 move segments in memory, 2.54 seconds should suffice
            await wait_for(self.wait_mem_empty(), timeout=5)
            if debug:
                elapsed = (time.ticks_ms() - start_time) / 1000
                logger.info(f"Waited for mem_empty {elapsed} seconds")
        self.fpga_cs.value(0)
        self.spi.write_readinto(command, response)
        self.fpga_cs.value(1)

        return response

    def init_steppers(self):
        """Initialize TMC2209 stepper drivers over UART."""
        esp32_cfg = self.cfg.esp32_cfg
        self.steppers = {}

        if not self.steppers_init:
            tmc_cfg = esp32_cfg["tmc2209"]
            failed = False
            for ax_name, mtr_id in tmc_cfg["mtr_ids"].items():
                try:
                    tmc = TMC_2209(
                        pin_en=esp32_cfg["stepper_cs"],
                        mtr_id=mtr_id,
                        uart_dct=tmc_cfg["uart"],
                    )
                    self.steppers[ax_name] = tmc

                except ConnectionFail:
                    logger.error(
                        f"Failed to initialize TMC2209 for {ax_name} (ID {mtr_id})."
                    )
                    failed = True
            self.steppers_init = not failed

    def flash_fpga(self, filename="sd/fpga/fpga.bit"):
        """Flash a bitstream file to the FPGA directly using SPI.

        Resets the FPGA and writes to it via SPI. The file is read in
        512-byte chunks to manage memory usage. Blocks execution until done.

        Args:
            filename (str): Path to the bitstream file (e.g. .bin) to be flashed.
        """
        # 1. Check if file exists
        try:
            os.stat(filename)
        except OSError:
            logger.error(f"Cannot flash FPGA: Bitstream file '{filename}' not found.")
            return False

        logger.info("Starting FPGA configuration sequence...")

        self.spi.deinit()
        self.spi.init()

        # 2. Hardware Reset Sequence
        self.fpga_cs.value(1)
        self.fpga_reset.value(0)
        time.sleep(0.5)  # Synchronous sleep
        self.fpga_cs.value(0)
        time.sleep(0.5)
        self.fpga_reset.value(1)
        time.sleep(0.5)

        # 3. Send 8 dummy clocks BEFORE the bitstream to wake up the FPGA.
        self.spi.write(b"\x00")

        # 4. Stream the file
        try:
            with open(filename, "rb") as f:
                while True:
                    chunk = f.read(512)
                    if not chunk:
                        break
                    self.spi.write(chunk)
        except OSError as e:
            logger.error(f"Error reading bitstream file: {e}")
            return False

        # 5. Finish configuration
        # Send a few clocks while CS is still low
        self.spi.write(bytes([0x00] * 10))

        # Raise CS and send >49 dummy clocks (13 bytes = 104 clocks)
        self.fpga_cs.value(1)
        time.sleep(0.5)
        self.spi.write(bytes([0x00] * 13))
        time.sleep(0.5)  # Give CDONE a moment to pull up

        # 6. Verification
        if self.fpga_done.value():
            logger.info("Successfully flashed FPGA!")
            return True
        else:
            logger.error("FPGA flash failed. CDONE pin not asserted.")
            return False

    async def synchronize(self, value=True):
        """Synchronize laser with phodiode.

        Args:
            value (bool): True to enable synchronization, False to disable.
        """
        if value:
            if not (await self.fpga_state)["synchronized"]:
                await self.enable_comp(
                    synchronize=True, laser0=False, laser1=False, polygon=False
                )
                await sleep(2)
                if not (await self.fpga_state)["synchronized"]:
                    return False
            return True
        else:
            await self.enable_comp(synchronize=False)
            return True

    def reset(self):
        "restart the FPGA by toggling the reset pin and initializing communication"
        # free all lines
        self.init_micropython()
        self.init_steppers()
        self.flash_fpga()
        length = Spi.word_bytes + Spi.command_bytes
        command = bytearray(length)
        response = bytearray(length)
        self.fpga_cs.value(0)
        self.spi.write_readinto(command, response)
        self.fpga_cs.value(1)

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
        return self.stepper_cs.value() == 0

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
        adr = self.cfg.esp32_cfg["i2c"]["digipot_addr"]
        # cam reset needs to be high to read the current value from the digipot
        # assumed this is done during intialization
        return list(self.i2c.readfrom_mem(adr, 0, 1))[0]

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
        adr = self.cfg.esp32_cfg["i2c"]["digipot_addr"]
        # cam reset needs to be high to write to the digipot
        # assumed this is done during initialization
        self.i2c.writeto_mem(adr, 0, bytes([val]))

    async def measure_facet_period_ms(self, samples=30, max_trials=10_000):
        """
        Measure the facet period in milliseconds and facet ID n_samples times.

        Parameters:
          samples: target samples per facet
          max_trials: maximum number of samples to reach target

        Returns: (facet_ms, facet_ids) both lists of length n_samples

        - facet_ms: each entry is period in ms for given facet_id
        - facet_id: each entry is facet ID
        """

        laz_tim = self.cfg.laser_timing

        # Sampling cadence to prevent oversampling
        dt_facet_ms = int(60 / (laz_tim["rpm"] * laz_tim["facets"] / 1000))
        facet_ms = np.zeros(max_trials)
        facet_id = np.zeros(max_trials)

        counts = np.zeros(laz_tim["facets"])

        for sample in range(max_trials):
            # improve distribution of samples
            if sample % 25 == 0:
                if np.min(counts) < samples:
                    delay = randint(0, 4 * dt_facet_ms)
                    await sleep_ms(delay)
                else:
                    return facet_ms[:sample], facet_id[:sample]
            f_ticks, f_id = await self.read_facet_ticks_and_id()
            counts[f_id] += 1
            facet_id[sample] = f_id
            facet_ms[sample] = f_ticks / (laz_tim["crystal_hz"] / 1000)
            delay = randint(int(0.5 * dt_facet_ms), dt_facet_ms)
            await sleep_ms(dt_facet_ms)
        logger.error("Measurement fails")
        return facet_ms, facet_id

    async def measure_facet_means(
        self,
        samples=30,
        max_trials=10_000,
    ):
        """
        Compute the mean period per facet in milliseconds using n_samples measurements.

          samples: target samples per facet
          max_trials: maximum number of samples to reach target

        Returns:
        - means_ms: list of length n_facets; each entry is mean period in ms or None if no samples
        """
        cur_sync = (await self.fpga_state)["synchronized"]
        if not cur_sync:
            await self.synchronize(True)

        facet_ms, facet_id = await self.measure_facet_period_ms(samples, max_trials)
        facets = self.cfg.laser_timing["facets"]

        mean_ms = [0] * facets

        for facet in range(facets):
            facet_ms_id = facet_ms[facet_id == facet]
            if len(facet_ms_id) > 0:
                mean_ms[facet] = np.mean(facet_ms_id)
                logger.debug(
                    f"Facet {facet}: n={facet_ms_id.size}, mean={mean_ms[facet]:.5f}, std={np.std(facet_ms_id):.5f}"
                )
            else:
                mean_ms[facet] = None
                logger.debug(f"Facet {facet}: n=0, mean=None, std=None")

        if not cur_sync:
            await self.synchronize(False)
        return mean_ms

    async def test_laserhead(self, shift=0):
        """
        Test laserhead by comparing jitter percentage of each facet against expected configuration.

        Measures the period for each facet multiple times, computes the jitter
        (ignoring statistical outliers) , and compares against the expected jitter percentage from configuration.

        Args:
        - shift (int, optional): Rotational offset used to map raw hardware facet IDs
          to calibrated logical facet IDs. Defaults to 0.

        Returns:
        - dict: A report containing the overall pass/fail status, global RPM stats,
          and detailed jitter/mean statistics for each logical facet.

        Notes:
        - If facet period is below half the expected period, test fails.
        - The `shift` parameter ensures that errors are reported against the correct
          physical facet based on the calibration table, regardless of boot orientation.
        """
        cur_sync = (await self.fpga_state)["synchronized"]
        if not cur_sync:
            await self.synchronize(True)

        laz_tim = self.cfg.laser_timing
        num_facets = laz_tim["facets"]
        expected_rpm = laz_tim["rpm"]
        exp_facet_ms = 60 / (expected_rpm * num_facets / 1000)

        # Initialize the report dictionary with the new RPM fields
        report = {
            "passed": True,
            "global_mean_ms": 0.0,
            "global_deviation_perc": 0.0,
            "expected_rpm": expected_rpm,
            "measured_rpm": 0,
            "facets": {},
        }

        # Request 200 samples for the percentile trim
        facet_ms, facet_ids = await self.measure_facet_period_ms(samples=200)

        # --- CASE 1: Global Mean Check (RPM Accuracy) ---
        global_mean_ms = float(np.mean(facet_ms))
        global_deviation_perc = float(
            abs(global_mean_ms - exp_facet_ms) / exp_facet_ms * 100
        )

        # Calculate the actual physical RPM based on the timings
        measured_rpm = round(60000 / (global_mean_ms * num_facets))

        # Populate global stats (rounded for clean JSON output)
        report["global_mean_ms"] = round(global_mean_ms, 4)
        report["global_deviation_perc"] = round(global_deviation_perc, 2)
        report["measured_rpm"] = measured_rpm

        if global_deviation_perc > 10.0:
            logger.error(
                f"Global timing failure: Mean period {global_mean_ms:.4f}ms "
                f"deviates {global_deviation_perc:.2f}% from expected {exp_facet_ms:.4f}ms."
            )
            report["passed"] = False
            report["error_reason"] = "Global RPM deviation too high"
            await self.synchronize(False)
            return report

        # --- CASE 2: Per-Facet Jitter Check ---
        for f_id in range(num_facets):
            raw_f_id = (f_id + shift) % num_facets
            facet_data = facet_ms[facet_ids == raw_f_id]
            n_samples = len(facet_data)

            # Default facet report
            facet_report = {
                "passed": False,
                "mean_ms": 0.0,
                "jitter_perc": 0.0,
                "samples_used": n_samples,
            }

            if n_samples < 50:
                logger.error(
                    f"Facet {f_id} (Hardware {raw_f_id}): Insufficient samples ({n_samples})."
                )
                report["passed"] = False
                report["facets"][f_id] = facet_report
                continue

            # 1. Sort the array in ascending order
            sorted_data = np.sort(facet_data)

            # 2. Drop 2% outliers from each end
            drop_count = int(n_samples * 0.02)
            clean_data = (
                sorted_data[drop_count:-drop_count] if drop_count > 0 else sorted_data
            )

            # 3. Calculate metrics
            clean_min = clean_data[0]
            clean_max = clean_data[-1]
            mean_val = np.mean(clean_data)

            min_frac_perc = (mean_val - clean_min) / mean_val * 100
            max_frac_perc = (clean_max - mean_val) / mean_val * 100
            total_jitter_perc = min_frac_perc + max_frac_perc

            # 4. Populate facet report (Rounded for consistency)
            facet_report["mean_ms"] = round(float(mean_val), 4)
            facet_report["jitter_perc"] = round(float(total_jitter_perc), 4)
            facet_report["samples_used"] = len(clean_data)

            jitter_limit = laz_tim["jitter_exp_perc"]
            is_facet_passed = total_jitter_perc <= jitter_limit
            facet_report["passed"] = is_facet_passed

            if not is_facet_passed:
                logger.error(
                    f"Facet {f_id}: Failed, Mean {mean_val:.4f} ms, "
                    f"Jitter {total_jitter_perc:.4f}% exceeds limit of {jitter_limit}%."
                )
                report["passed"] = False
            else:
                logger.info(
                    f"Facet {f_id}: Passed, Mean {mean_val:.4f} ms, "
                    f"Jitter {total_jitter_perc:.4f}%."
                )

            report["facets"][f_id] = facet_report

        if not cur_sync:
            await self.synchronize(False)

        return report


@syncable
class ESP32HostSync(ESP32Host):
    def __init__(self, sync=True):
        super().__init__()
