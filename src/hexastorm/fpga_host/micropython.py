from asyncio import sleep
import logging
import sys
from machine import Pin, SPI, I2C, SoftSPI


from tmc.uart import ConnectionFail
from tmc.stepperdriver import TMC_2209

from .interface import BaseHost
from ..config import Spi

if sys.implementation.name == "micropython":
    from winbond import W25QFlash


class ESP32Host(BaseHost):
    """
    Host interface to interact with the FPGA using micropython.
    """

    def __init__(self):
        super().__init__(test=False)
        self.steppers_init = False
        self.spi_tries = 1e5
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
                    logging.error(
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
        self.flash_cs.init(Pin.IN)

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
            raise ValueError(
                "enable_steppers must be a boolean value (True or False)"
            )

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
