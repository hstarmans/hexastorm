"""Platform and SPI Configuration

This module defines constants and utilities for the SPI communication protocol,
as well as platform-level configuration settings used for FPGA synthesis and firmware setup.

- `Spi`: SPI protocol structure (commands, instructions, state bits, and word size logic)
- `PlatformConfig`: High-level platform configuration object for both test and production modes
"""

import logging
import sys
from collections import OrderedDict
from math import ceil, radians
from typing import ClassVar

if sys.implementation.name == "micropython":
    from ulab import numpy as np

    MICROPYTHON = True
else:
    import json
    from pathlib import Path

    import numpy as np

    MICROPYTHON = False

try:
    from camera import FrameSize, GrabMode, PixelFormat
except ImportError:

    class FrameSize:
        UXGA = 15

    class PixelFormat:
        JPEG = 4

    class GrabMode:
        LATEST = 1


logger = logging.getLogger(__name__)


class Spi:
    """Holds constants and types for the SPI communication protocol."""

    command_bytes = 1
    word_bytes = 8
    move_instruction: ClassVar[dict[str, int]] = {"instruction": 1, "ticks": 7}

    class Commands:
        """SPI protocol command. Each command is followed by a word."""

        empty = 0
        write = 1
        read = 2
        debug = 3
        flush = 4
        start = 5
        stop = 6

    class Instructions:
        """Instruction types encoded in SPI words. Each word can contain a subcommand."""

        move = 1
        write_pin = 2
        scanline = 3
        last_scanline = 4
        set_fan = 5
        set_spindle = 6

    class State:
        """State word returned by SPI. Each bit represent a specific status flag."""

        full = 0
        parsing = 1
        error = 2
        empty = 3

    @staticmethod
    def words_move(hdl_cfg):
        """Returns the number of words required for a single move instruction."""
        bytes_move = (
            sum(Spi.move_instruction.values())
            + hdl_cfg["motors"] * hdl_cfg["pol_degree"] * Spi.word_bytes
        )
        bytes_move += bytes_move % Spi.word_bytes
        return ceil(bytes_move / Spi.word_bytes)


class Camera:
    """Central hardware and optical specs for supported cameras (OV2640, OV2311)."""

    OV2640: ClassVar[dict] = {
        "name": "OV2640",
        "width": 1600,
        "height": 1200,
        "pixel_size_um": 2.2,
        "svga_pixel_size_um": 4.4,
        "i2c_addr": 0x30,
        "pins": {
            "d0": 12,
            "d1": 14,
            "d2": 21,
            "d3": 13,
            "d4": 11,
            "d5": 10,
            "d6": 9,
            "d7": 8,
            "pclk": 38,
            "vsync": 41,
            "href": 40,
            "xclk": 39,
            "xclk_freq": 10_000_000,
            "pwdn": -1,
            "reset": 42,
        },
        "settings": {
            "exposure_ms": 200,
            "frame_size": FrameSize.UXGA,
            "pixel_format": PixelFormat.JPEG,
            "jpeg_quality": 85,
            "fb_count": 1,
            "grab_mode": GrabMode.LATEST,
        },
    }

    OV2311: ClassVar[dict] = {
        "name": "OV2311",
        "width": 1600,
        "height": 1300,
        "pixel_size_um": 3.0,
        "line_time_ms": 0.0189,
    }

    DEFAULT_PIXEL_SIZE_UM = OV2311["pixel_size_um"]

    @classmethod
    def esp32_camera_cfg(cls) -> dict:
        """Return a fresh flat camera config dict for ``PlatformConfig.esp32_cfg``."""
        cfg = dict(cls.OV2640["pins"])
        cfg["settings"] = dict(cls.OV2640["settings"])
        return cfg


# Pure-NumPy optical displacement kernel
def displacement_at_frac(frac, params):
    """Calculate transversal beam displacement [mm] at facet rotation fraction ``frac``."""
    i_max = 180 / params["facets"]
    angle = np.radians(-2 * i_max * frac + i_max)

    sin_angle = np.sin(angle)
    sin_angle_sq = sin_angle**2
    n_sq = params["n"] ** 2
    thickness = params["inradius"] * 2

    return (
        thickness
        * sin_angle
        * (1 - ((1 - sin_angle_sq) / (n_sq - sin_angle_sq)) ** 0.5)
    )


def displacement_kernel(pixel, params):
    """Legacy pixel-domain wrapper around :func:`displacement_at_frac`."""
    span = float(params["end_frac"]) - float(params["start_frac"])
    frac = float(pixel) * span / float(params["bitsinscanline"])
    return displacement_at_frac(frac, params)


class PlatformConfig:
    """
    Holds platform configuration.
    """

    def __init__(self, test=False):
        """
        Initialization follows one of two routes:

            - **Test Mode** (`test=True`):
            - You define low-level timing values directly (`facet_ticks`, `scanline_ticks`)
            - Useful for simulation or FPGA-level tuning

            - **Production Mode** (`test=False`):
            - You define real-world timing values (`rpm`, `start_frac`, `end_frac`, `pd_debounce_us`, etc.)
            - The system computes derived timing parameters

            The final result is stored in `self.laser_timing`, a flat dictionary of all configuration values.
        """
        self.test = test
        self._hdl_cfg = None
        if test:
            self.laser_timing = {
                "rpm": 1000,
                "facet_ticks": 20,
                "scanline_ticks": 12,
            }
        else:
            self.laser_timing = {
                "rpm": 3000,
                "spinup_time": 1.5,
                "stable_time": 1.125,
                "start_frac": 0.35,
                "end_frac": 0.7,
                "pd_debounce_us": 20.0,
            }
        self.update_laser_timing()
        self.motor_cfg = self._init_motor_cfg()
        self.laser_bits = 1  # enables adding pwm to laser (not widely tested)

    @property
    def paths(self):
        """
        Standardized output directories for files generated during tests/calibration.
        Returns a dictionary of pathlib.Path objects. Auto-creates directories if they don't exist.
        """
        if MICROPYTHON:
            logger.error(
                "The ESP32 doesn't handle these files natively. Skipping path setup."
            )
            return {}

        project_root = Path(__file__).resolve().parent.parent.parent
        output_base = project_root / "output"

        dirs = {
            "base": output_base,
            "calibration": output_base / "calibration",
            "images": output_base / "images",
            "patterns": output_base / "patterns",
            "svgs": output_base / "svgs",
            "debug": output_base / "debug",
        }

        # Auto-create the directories on the Host PC
        for path in dirs.values():
            path.mkdir(parents=True, exist_ok=True)

        return dirs

    def load_latest_calibration(self):
        """Helper to fetch the latest calibration dictionary from disk."""
        if MICROPYTHON:
            logger.error("Calibration correction not supported in MicroPython.")
            return {}

        history_path = self.paths["calibration"] / "calibration_history.json"
        with open(history_path) as f:
            history = json.load(f)

        latest = history[-1]
        logger.info(f"Applied calibration from: {latest.get('timestamp')}")
        return latest["facets"]

    def get_visible_pixels(self):
        """Retrieves pixels visible on camera"""
        if MICROPYTHON:
            logger.error("Calibration correction not supported in MicroPython.")
            return {}

        history_file = self.paths["calibration"] / "scan_visibility.json"
        with open(history_file) as f:
            history = json.load(f)
            last_record = history[-1]
            visible_pixels = last_record.get("visible_pixels", [])
        return visible_pixels

    def compute_photodiode_scan_corrections(
        self, facet_period_ms: list[float] | np.ndarray, k_scale: float = 1.628
    ) -> dict[str, dict[str, float]]:
        """
        Computes zero-camera feed-forward along-scan corrections (in mm) directly from
        photodiode timing measurements per facet.

        Physics & Derivation Context:
            The mapping between photodiode timing differences and along-scan spatial shift
            is not simply `v_scan(0) * delta_t` because of the scanline duty cycle geometry:

            1. Center Scan Speed:
               The transversal displacement through the rotating prism follows Snell's Law:
               `tau(I) = T * sin(I) * (1 - sqrt((1 - sin(I)^2) / (n^2 - sin(I)^2)))`
               Near line center (I = 0 deg), d(tau)/dI = T * (1 - 1/n) = 30.0 mm * (1 - 1/1.49) = 9.866 mm/rad.
               At 3000 RPM (omega = 314.159 rad/s), the scan speed is:
               `v_scan(0) = (d(tau)/dI) * omega = 9.866 * 314.159 = 3.10 mm/ms (3.10 um/us)`.

            2. Duty Cycle & Trigger Position:
               The photodiode triggers at the start of the facet sweep (frac = 0.0), whereas
               the active laser exposure window runs from `start_frac = 0.35` to `end_frac = 0.70`,
               centered at `center_frac = (0.35 + 0.70) / 2 = 0.525`.
               The timing deviation measured across the full facet sweep accumulates proportionally
               at the center of the active write window:
               `delta_t_effective = delta_t * center_frac`.
               Therefore, the effective scale factor is:
               `k_scale = v_scan(0) * center_frac = 3.10 mm/ms * 0.525 = 1.628 mm/ms (1.628 um/us)`.

            3. First-Order Approximation & Residuals:
               Total along-scan deviation is composed of:
               `Delta_x = Delta_x_timing(delta_t) + Delta_x_mechanical_wedge`.
               This method corrects for dynamic/rotational timing variations Delta_x_timing (slashing
               jitter by >50%), while static individual facet wedge errors and non-linear velocity
               variations across the sweep remain as smaller residuals.

        Args:
            facet_period_ms: Array or list of measured periods for each facet in milliseconds.
            k_scale: Scale factor converting timing offset (ms) to spatial shift (mm).
                     Defaults to 1.628 mm/ms (derived from v_scan(0) * center_frac).

        Returns:
            dict: Correction dictionary formatted for `get_optical_params(correction=...)`.
        """
        periods = np.array(facet_period_ms, dtype=float)
        p_median = float(np.median(periods))
        delta_t_ms = periods - p_median

        correction_dict = {}
        for i, dt in enumerate(delta_t_ms):
            correction_dict[str(i)] = {
                "scan": float(k_scale * dt),
                "orth": 0.0,
            }
        return correction_dict

    def get_optical_params(
        self, correction=False, exposures=4, samplegridsize=0.01
    ):
        """
        Returns a dictionary of physical parameters, including calculated Lanewidth.

        correction: If True, loads empirically derived corrections from calibration_history.json.
        exposures: Number of exposures per facet, used to calculate stepsperline for stage speed.
        samplegridsize: Fallback software raster pitch in mm (default 0.01 mm = 10 um) used
            for vector-to-raster fallback (.svg/.ps) and debug plot reconstruction. True bitmaps
            use their native physical pixelsize directly.
        """
        if correction is True:
            correction_data = self.load_latest_calibration()
        elif isinstance(correction, dict):
            correction_data = correction
            logger.info("Applied custom calibration dictionary.")
        else:
            correction_data = {}  # Handles False, None, etc.
        laz_tim = self.laser_timing
        mtr_cfg = self.motor_cfg

        # need steps per line to calculate stage speed
        params = {"stepsperline": 1 / exposures}  # 0.25 is four exposures per line

        params.update(
            {
                # angle [radians]
                "tiltangle": radians(90),  # Static polygonal tilt angle
                # r: Inradius of the polygon [mm] (T = 2r)
                "inradius": 15.0,
                # Refractive index of the prism material (e.g. Quartz
                "n": 1.49,
                # Number of vertices/facets on the polygon, must be even
                "facets": float(laz_tim["facets"]),
                "crystal_hz": float(laz_tim["crystal_hz"]),
                "facet_ticks": int(laz_tim["facet_ticks"]),
                "scanline_ticks": int(laz_tim["scanline_ticks"]),
                "start_frac": float(laz_tim["start_frac"]),
                "end_frac": float(laz_tim["end_frac"]),
                # Polygon rotation frequency (Hz)
                "rotationfrequency": laz_tim["rpm"] / 60,
                # Radius of the laser beam [mm]
                "laser_radius_mm": 0.015,
                # Platform dimensions [mm]
                "pltfxsize": 200.0,
                "pltfysize": 200.0,
                # Sample dimensions (calculated dynamically from image)
                "samplexsize": 0.0,
                "sampleysize": 0.0,
                # Optical resolution / Grid size [mm]
                "samplegridsize": float(samplegridsize),
                "facetsinlane": 0.0,
                # Stage Speed [mm/s]: Derived from RPM and Facets to ensure correct aspect ratio
                "stagespeed": (
                    (
                        params["stepsperline"]
                        / mtr_cfg["steps_mm"][mtr_cfg["orth2lsrline"]]
                    )
                    * (laz_tim["rpm"] / 60)
                    * laz_tim["facets"]
                ),
                "downsamplefactor": 1.0,
                # resist
                "positiveresist": 0.0,
            }
        )

        lanewidth = abs(
            displacement_at_frac(laz_tim["end_frac"], params)
            - displacement_at_frac(laz_tim["start_frac"], params)
        )
        params["lanewidth"] = lanewidth

        # Derive software raster sample count from physical samplegridsize (10 um -> 560 samples)
        raw_samples = max(1, ceil(lanewidth / float(samplegridsize)))
        if laz_tim["scanline_ticks"] >= 8:
            raster_samples = min(
                ((raw_samples + 7) // 8) * 8, int(laz_tim["scanline_ticks"])
            )
        else:
            raster_samples = int(laz_tim["scanline_ticks"])
        params["bitsinscanline"] = int(raster_samples)

        num_facets = int(laz_tim["facets"])

        for i in range(num_facets):
            if correction_data != {}:
                params[f"f{i}_scan"] = correction_data[str(i)]["scan"]
                params[f"f{i}_orth"] = correction_data[str(i)]["orth"]
            else:
                params[f"f{i}_scan"] = 0.0
                params[f"f{i}_orth"] = 0.0

        # Store metadata so writers (e.g. write_binary_file) can encode it in the filename
        params["exposures"] = int(exposures)
        params["optical_correction"] = correction_data != {}
        return params

    @property
    def camera_cfg(self):
        """Hardware and optical specifications for supported camera sensors."""
        return {
            "ov2640": Camera.OV2640,
            "ov2311": Camera.OV2311,
        }

    @property
    def esp32_cfg(self):
        """Connections to esp32S3."""
        return {
            "tmc2209": {
                "mtr_ids": {"x": 0, "y": 1, "z": 2},
                "uart": {
                    "id": 2,
                    "ctor": {
                        "baudrate": 115200,
                        "tx": 15,
                        "rx": 7,
                    },
                    "init": {
                        "bits": 8,
                        "parity": None,
                        "stop": 1,
                    },
                },
            },
            "camera": Camera.esp32_camera_cfg(),
            "i2c": {
                "scl": 17,
                "sda": 18,
                "freq": int(1e5),
                "digipot_addr": 0x28,
                "cam_addr": Camera.OV2640["i2c_addr"],
            },
            "spi": {
                "sck": 6,
                "miso": 5,
                "mosi": 4,
                "phase": 1,
                "polarity": 0,
                "baudrate": int(5e6),
            },
            "stepper_cs": 16,  # enable pin stepper motors
            "clk": {
                "pin": 16,
                "duty": 512,  # 50 percent
            },
            "fpga": {
                "cs": 2,
                "reset": 43,
                "done": 44,
                "mem_full": 1,
            },
            "device": "/dev/ttyACM0",
        }

    @property
    def ice40_cfg(self):
        """Required for LatticeICE40Platform."""
        return {
            "device": "iCE40UP5K",
            "package": "SG48",
            "default_clk": "SB_HFOSC",
            "hfosc_div": "test" if self.test else 1,
            "clks": {0: 48, 1: 24, 2: 12, 3: 6, "esp32s3": 27.978142, "test": 1},
        }

    @property
    def hdl_cfg(self):
        """Required for amaranth synthesis."""
        if self._hdl_cfg is not None:
            return self._hdl_cfg

        cfg = {
            "single_line": False,
            "motor_freq": 1e6,  # motor move interpolation freq in Hz
            "move_ticks": 10_000,  # maximum ticks in move segment
            "direction": 0,  # axis parallel to laser, here x
            "motors": len(self.motor_cfg["steps_mm"]),
            "motor_divider": pow(2, 8),
            "pol_degree": 2,
            "mem_width": Spi.word_bytes * 8,
            "motor_debug": "ticks_in_facet",
        }
        if self.test:
            cfg.update(
                {
                    "test": True,
                    "lines_chunk": 2,
                    "fifo_margin_words": 4,
                }
            )
        else:
            mem_depth = int((114 * 1000) / (Spi.word_bytes * 8))
            cfg.update(
                {
                    "test": False,
                    # listed max 120 kbit, practical 114 kbit
                    # you use EBR / sysMEM (Block RAM) 120 kbit
                    "mem_depth": mem_depth,
                    "lines_chunk": 50,
                    "fifo_margin_words": 600,
                }
            )
            assert mem_depth > cfg["fifo_margin_words"]
        # Choose bit used for stepping
        if cfg["pol_degree"] == 3:
            cfg["bit_shift"] = 40
        elif cfg["pol_degree"] == 2:
            cfg["bit_shift"] = 25
        else:
            raise ValueError("Only polynomial orders 2 and 3 are supported")

        cfg["words_move"] = Spi.words_move(cfg)
        if self.test:
            cfg["mem_depth"] = cfg["words_move"] * 2 + 1

        self._hdl_cfg = type("Hdl_cfg", (), cfg)()
        return self._hdl_cfg

    def _init_motor_cfg(self):
        """Returns the steps per mm and axis orthogonal to laserline."""

        if self.test:
            steps = OrderedDict([("x", 400), ("y", 400)])
            offset_mm = OrderedDict(
                [
                    ("x", 0),
                    ("y", 0),
                    ("z", 0),
                ]
            )
        else:
            # offset mm and steps are overwritten by esp32_hexastorm!
            steps = OrderedDict(
                [
                    ("x", 100),
                    ("y", 100),
                    ("z", 100),
                ]
            )
            offset_mm = OrderedDict(
                [
                    ("x", 10),
                    ("y", 10),
                    ("z", -10),
                ]
            )
        return {
            "homing_dir": {"x": -1, "y": -1, "z": 1},
            "steps_mm": steps,
            "offset_mm": offset_mm,
            "min_mm": {"x": 0.0, "y": 0.0, "z": 0.0},
            "max_mm": {"x": 200.0, "y": 200.0, "z": 100.0},
            "orth2lsrline": "x",
        }

    def update_laser_timing(self):
        """Update laser timing parameters."""
        # Unpack into locals for brevity
        rpm = self.laser_timing["rpm"]
        facets = 4

        poly_hz = rpm / 60

        if self.test:
            facet_ticks = int(self.laser_timing["facet_ticks"])
            scanline_ticks = int(self.laser_timing["scanline_ticks"])

            crystal_hz = round(facet_ticks * facets * poly_hz)
            spinup_time = 10 / crystal_hz
            stable_time = 5 * facet_ticks / crystal_hz
            start_frac = 2 / facet_ticks
            end_frac = scanline_ticks / facet_ticks + start_frac
            assert facet_ticks == round(crystal_hz / (poly_hz * facets))
            pd_trigger_ticks = 1
            pd_rearm_ticks = 2
            jitter_sync_ticks = 2
        else:
            spinup_time = self.laser_timing["spinup_time"]
            stable_time = self.laser_timing["stable_time"]
            start_frac = self.laser_timing["start_frac"]
            end_frac = self.laser_timing["end_frac"]

            crystal_hz = self.ice40_cfg["clks"][self.ice40_cfg["hfosc_div"]] * 1e6
            facet_ticks = round(crystal_hz / (poly_hz * facets))
            scanline_ticks = round(facet_ticks * (end_frac - start_frac))
            pd_debounce_us = self.laser_timing.get("pd_debounce_us", 20.0)
            pd_trigger_ticks = max(1, round(pd_debounce_us * 1e-6 * crystal_hz))
            pd_rearm_ticks = pd_trigger_ticks
            # jitter allowed for synchronization 1% is larger than
            # jitter allowed for exposure 0.2%
            jitter_sync_ticks = round(0.01 * facet_ticks)

        jitter_exp_perc = 0.2
        spinup_ticks = round(spinup_time * crystal_hz)
        stable_ticks = round(stable_time * crystal_hz)

        motor_period = int(crystal_hz / (poly_hz * 6 * 2))

        # Sanity checks
        if scanline_ticks <= 0:
            raise ValueError("scanline_ticks invalid")
        if end_frac >= 1 - (jitter_sync_ticks + 1) / facet_ticks:
            raise ValueError("Invalid settings, end_frac too high")

        # Update dictionary
        self.laser_timing.update(
            {
                "facets": facets,
                "crystal_hz": crystal_hz,
                "start_frac": start_frac,
                "end_frac": end_frac,
                "facet_ticks": facet_ticks,
                "scanline_ticks": scanline_ticks,
                "spinup_ticks": spinup_ticks,
                "stable_ticks": stable_ticks,
                "jitter_sync_ticks": jitter_sync_ticks,
                "jitter_exp_perc": jitter_exp_perc,
                "motor_period": motor_period,
                "photodiode_trigger_ticks": pd_trigger_ticks,
                "photodiode_rearm_ticks": pd_rearm_ticks,
            }
        )
