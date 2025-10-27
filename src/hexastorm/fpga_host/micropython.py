from asyncio import sleep, sleep_ms, Event
import time
import logging
import sys
from random import randint
from machine import Pin, SPI, I2C, SoftSPI


from ulab import numpy as np
from tmc.uart import ConnectionFail
from tmc.stepperdriver import TMC_2209

from .syncwrap import syncable
from .interface import BaseHost
from ..config import Spi


if sys.implementation.name == "micropython":
    from winbond import W25QFlash

logger = logging.getLogger(__name__)


@syncable
class ESP32Host(BaseHost):
    """
    Host interface to interact with the FPGA using micropython.
    """

    def __init__(self, sync=True):
        super().__init__(test=False)
        self.steppers_init = False
        self.init_micropython()
        self.reset()
        self.init_steppers()

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
        self._mem_full_event = Event()
        self._mem_full = Pin(cfg["mem_full"], Pin.IN)

        # initialize event based on current pin state
        if not self._mem_full.value():
            self._mem_full_event.set()

        # attach an interrupt handler
        self._mem_full.irq(
            trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=self._mem_full_callback
        )

    def init_steppers(self):
        """Configure TMC2209 stepper drivers over UART.

        Sets current, interpolation, and microstepping parameters for each stepper motor.
        Logs failures if drivers are not detected. Only runs once per session.
        """
        esp32_cfg = self.cfg.esp32_cfg

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
                    for key, value in tmc_cfg["settings"]:
                        setattr(tmc, key, value)
                except ConnectionFail:
                    logger.error(
                        f"Failed to initialize TMC2209 for {ax_name} (ID {mtr_id})."
                    )
                    failed = True
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
        self.flash_cs.value(1)
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
            cs=self.flash_cs,
            baud=cfg["spi"]["baudrate"],
            software_reset=True,
        )

        with open(filename, "rb") as infile:
            buffsize = f.BLOCK_SIZE
            blocknum = 0
            while True:
                buf = infile.read(buffsize)
                if blocknum % 10 == 0:
                    logger.info(f"Writing block {blocknum}.")
                f.writeblocks(blocknum, buf)
                if len(buf) < buffsize:
                    logger.info(f"Final block {blocknum}")
                    break
                blocknum += 1
        self.reset()
        logger.info("Flashed fpga.")

    def reset(self):
        "restart the FPGA by toggling the reset pin and initializing communication"
        # free all lines
        self.spi.deinit()
        self.flash_cs.init(Pin.IN)

        self.fpga_reset.value(0)
        time.sleep(1)
        self.fpga_reset.value(1)
        time.sleep(1)
        self.init_micropython()
        length = Spi.word_bytes + Spi.command_bytes
        command = bytearray(length)
        response = bytearray(length)
        self.fpga_cs.value(0)
        self.spi.write_readinto(command, response)
        self.fpga_cs.value(1)

    def _mem_full_callback(self, pin):
        # Pin changed: if memory not full, set event; if full, clear event
        if not pin.value():  # assuming 0 = not full
            self._mem_full_event.set()
        else:
            self._mem_full_event.clear()

    @property
    def mem_full(self):
        """
        Returns whether the memory buffer is full (boolean).
        """
        return self._mem_full.value()

    async def await_mem_empty(self):
        """
        Wait until the memory buffer is empty.
        """
        # Wait until event is set (memory not full)
        await self._mem_full_event.wait()

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
        adr = self.cfg.esp32_cfg["i2c"]["digipot_addr"]
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
        logging.error("Measurement fails")
        return facet_ms, facet_id

    async def facet_mean(
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
        facet_ms, facet_id = await self.measure_facet_period_ms(samples, max_trials)
        facets = self.cfg.laser_timing["facets"]

        mean_ms = [0] * facets

        for facet in range(facets):
            facet_ms_id = facet_ms[facet_id == facet]
            if len(facet_ms_id) > 0:
                mean_ms[facet] = np.mean(facet_ms_id)
                logger.info(
                    f"Facet {facet}: n={facet_ms_id.size}, mean={mean_ms[facet]:.5f}, std={np.std(facet_ms_id):.5f}"
                )
            else:
                mean_ms[facet] = None
                logger.info(f"Facet {facet}: n=0, mean=None, std=None")
        return mean_ms

    async def test_laserhead(self):
        """
        Test laserhead by comparing jitter percentage of each facet period against expected configuration.

        Measures the period for each facet multiple times, computes the jitter
        and compares against the expected jitter percentage from configuration.

        Returns:
           - True if jitter is within expected limits.
           - False if jitter exceeds expected limits.

        Notes:
        - If facet period is below half the expected period, test fails.
        """
        laz_tim = self.cfg.laser_timing

        exp_facet_ms = 60 / (laz_tim["rpm"] * laz_tim["facets"] / 1000)

        facet_ms, _ = await self.measure_facet_period_ms()

        if np.min(facet_ms) < (0.5 * exp_facet_ms):
            logger.error(
                f"Facet period {np.min(facet_ms)} ms is less than half the expected {exp_facet_ms} ms."
            )
            return False
        else:
            mean_facet_ms = np.mean(facet_ms)
            min_frac_perc = (mean_facet_ms - np.min(facet_ms)) / mean_facet_ms * 100
            max_frac_perc = (np.max(facet_ms) - mean_facet_ms) / mean_facet_ms * 100
            total_frac_perc = min_frac_perc + max_frac_perc
            if total_frac_perc > laz_tim["jitter_exp_perc"]:
                logger.error(
                    f"Jitter % not compliant {total_frac_perc:.3f} > {laz_tim['jitter_exp_perc']}"
                )
                return False
            else:
                logger.info(
                    f"Jitter [lower, upper] % is [{min_frac_perc:.3f}, {max_frac_perc:.3f}]"
                )
                return True
