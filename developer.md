# Developer Reference

This document covers the wire-level FPGA SPI/FIFO protocol, motion math, operational timing parameters, laserhead stability & optical calibration workflow, and camera setups (target onboard ESP32-S3 OV2640 and legacy Raspberry Pi OV2311 bench).

---

# FPGA Architecture & SPI Protocol

The host (ESP32-S3 or `MockHost`) communicates with `SPIParser` ([src/hexastorm/core.py](src/hexastorm/core.py)) over SPI Mode 1 (`polarity=0, phase=1`, MSB-first at 5 MHz). Every SPI transfer starts with an **8-bit command byte** followed by a **64-bit (8-byte) data word** (`Spi.command_bytes = 1`, `Spi.word_bytes = 8` in [src/hexastorm/config.py](src/hexastorm/config.py)).

For `write` commands, `SPIParser` pushes complete multi-word instructions into a `TransactionalizedFIFO` in iCE40 Block RAM (`depth = 1781` words in production, `width = 64` bits) and issues `write_commit` only when the full instruction has arrived. While `self.parse` is active (`1` on reset), `Dispatcher` pops committed instructions and routes them to `Laserhead`, `Polynomial`, or `HardwarePWM`.

When `fifo.space_available <= fifo_margin_words` (`600` words in production), the hardware `fifo_full` output pin and status bit `full` are asserted so the host pauses streaming before an overflow can occur.

## SPI Commands (`Spi.Commands`)

| Opcode | Name | Request Word (64-bit) | Response Word (64-bit) | Description |
| :--- | :--- | :--- | :--- | :--- |
| `0` | `empty` | Ignored | Previous `word_to_send` | No-op / idle frame |
| `1` | `write` | Instruction header or payload word | Status + pin state | Append 64-bit word to current FIFO packet |
| `2` | `read` | Ignored (`0`) | Status + pin state | Read status flags, limit switches, photodiode & sync state |
| `3` | `debug` | Ignored (`0`) | `facet_period_ticks` | Read `[ticks_per_facet (56-bit) \| facet_id (8-bit)]` |
| `4` | `flush` | Ignored (`0`) | Previous `word_to_send` | Reset FIFO pointers, `words_rec`, and parser `error_word` |
| `5` | `start` | Ignored (`0`) | Previous `word_to_send` | Enable FIFO execution (`parse = 1`) |
| `6` | `stop` | Ignored (`0`) | Previous `word_to_send` | Pause FIFO execution (`parse = 0`) |

### Status & Pin State Word (`read` / `write` response)

The 16-bit `state_word` returned in the low 2 bytes of `word_to_send` (`bytes[-1]` = `status_byte`, `bytes[-2]` = `pin_byte` on the host) packs:

* **Status byte (`Spi.State`):**
  * Bit `0` (`full`): `1` when `fifo.space_available <= fifo_margin_words`
  * Bit `1` (`parsing`): `1` when `Dispatcher` is enabled (`parse`)
  * Bit `2` (`error`): `error_other | error_word` (illegal opcode, `Laserhead` sync/photodiode error, or parser error)
  * Bit `3` (`empty`): `1` when the committed FIFO is empty
* **Pin state byte (`pin_state`):**
  * Bits `0 .. motors-1`: Stepper limit switches (`x`, `y`, `z` in order of `motor_cfg["steps_mm"]`)
  * Bit `motors`: `photodiode_trigger` (high if photodiode went low during the last measurement window)
  * Bit `motors + 1`: `synchronized` (`1` when prism rotation is locked to photodiode triggers)

---

## FIFO Instructions (`Spi.Instructions`)

Each instruction stored in the FIFO begins with an 8-bit opcode in bits `0..7` of its first 64-bit word.

### `1` — `move` (Coordinated Polynomial Spline Motion)

Evaluates a discrete polynomial per motor axis at `motor_freq = 1 MHz` inside `Polynomial` ([src/hexastorm/movement.py](src/hexastorm/movement.py)):

```text
pos(t) = c·t³ + b·t² + a·t
```

Where `t` is the 1 MHz tick index (`0 <= t < tick_limit`, max `move_ticks = 10 000` ticks per segment) and `pos(t)` is a fixed-point Q-format accumulator (`bit_shift = 25` for `pol_degree = 2`, `40` for `pol_degree = 3`). Toggling bit `bit_shift` of the position accumulator emits one step pulse, and the sign bit of the velocity accumulator drives `dir` (positive = `1`, negative = `0`). Fractional sub-step remainders are preserved across consecutive `move` segments.

To avoid hardware multipliers/DSPs on the iCE40UP5K, the host pre-computes **discrete forward differences** (`D1, D2, D3`):

```text
D1 = a + b + c
D2 = 2b + 6c
D3 = 6c
```

| Word | Bits | Field | Description |
| :--- | :--- | :--- | :--- |
| Word `0` | `0..7` | `opcode = 0x01` | `Spi.Instructions.move` |
| Word `0` | `8..63` | `tick_limit` | 56-bit segment duration in 1 MHz ticks (`≤ 10 000`) |
| Words `1 .. motors × pol_degree` | `0..63` | `coeff` | Signed 64-bit forward differences grouped by axis (`[m0_D1, m0_D2, ..., m1_D1, ...]`) |

*Note: Maximum step rate must stay below half the 1 MHz motor update rate (Nyquist limit) so at most one step occurs per tick.*

### `2` — `write_pin` (1 word)

Updates the synchronous 8-bit output control register `Dispatcher.pins`:

| Bit | Signal | Description |
| :--- | :--- | :--- |
| `0` | `laser0` | Manual override for laser channel 0 (`lasers_in[0]`) |
| `1` | `laser1` | Manual override for laser channel 1 (`lasers_in[1]`) |
| `2` | `enable_prism` | Manual prism motor enable (`enable_prism_in`) |
| `3` | `synchronize` | Enable/disable `Laserhead` photodiode synchronization loop |
| `4` | `singlefacet` | Restrict scanline exposure to a single prism facet |
| `5` | `led0` | Debug LED 0 (blue, active-low hardware pin) |
| `6` | `led1` | Debug LED 1 (green, active-low hardware pin) |
| `7` | `led2` | Debug LED 2 (red, active-low hardware pin) |

### `3` — `scanline` (Event-Driven Laser Scanline, `1 + event_words` words)

Wait-free variable-length event stream processed by `Laserhead` ([src/hexastorm/lasers.py](src/hexastorm/lasers.py)):

* **Word `0` (64-bit Scanline Header, little-endian):**
  * Bits `0..7`: `opcode = 0x03` (`Spi.Instructions.scanline`)
  * Bit `8`: `direction` (`0` = backward, `1` = forward for orthogonal stage step `dir`)
  * Bits `9..24`: `event_words` (16-bit count of 64-bit event words following this header; `0` for blank lines)
  * Bits `25..63`: `step_half_period` (39-bit FPGA clock tick half-period for orthogonal stage motion during `DATA_RUN`; `0` = stationary)
* **Words `1 .. event_words` (two 32-bit events per 64-bit word, little-endian):**
  * Event 0 in bits `0..31`: `[8-bit intensity (0..7) | 24-bit tick (8..31)]`
  * Event 1 in bits `32..63`: `[8-bit intensity (32..39) | 24-bit tick (40..63)]`
  * Unused trailing slot (when event count is odd): sentinel `tick = 0xFFFFFF`, `intensity = 0` (`0xFFFFFF00`).

During `DATA_RUN`, `line_tick` starts at `0` at `start_frac × facet_ticks` and increments at the full FPGA clock (`24 MHz`, up to `scanline_ticks = 42 000`). Both event-driven laser switching and orthogonal stage stepping (`stephalfperiod`) run directly at `24 MHz`. Each event's 8-bit `intensity` drives both laser outputs synchronously via a 1st-order `24 MHz` Sigma-Delta PDM stage: `0` holds the diodes off, `255` holds them continuously ON (100% binary, zero switching), and `1..254` modulates at `24 MHz` (`intensity / 256` duty cycle, suited for fast laser switches such as the iC-Haus iC-HKB).

### `4` — `last_scanline` (1 word)

Marks the end of an exposure pass:

* Word `0`: `[0x04, 0, 0, 0, 0, 0, 0, 0]`.
* Clears `process_lines`, asserts `expose_finished`, and returns `Dispatcher` to idle without forcing the prism motor to spin down.

### `5` & `6` — `set_fan` / `set_spindle` (1 word)

Sets the 8-bit duty cycle (`0..255` in bits `8..15`) on `HardwarePWM` ([src/hexastorm/pwm.py](src/hexastorm/pwm.py)):
* `0x05` (`set_fan`): 12-bit prescaler (`≈ 22.9 Hz` at 24 MHz clock)
* `0x06` (`set_spindle`): no prescaler (`93.75 kHz` at 24 MHz clock)

---

## Operational Parameters (`PlatformConfig`)

Default production values in [src/hexastorm/config.py](src/hexastorm/config.py):

| Parameter | Default | Description |
| :--- | :--- | :--- |
| `rpm` | `3000` | Polygon prism rotational speed (RPM) |
| `facets` | `4` | Number of prism facets |
| `start_frac` | `0.35` | Fraction of facet period where the active scanline begins |
| `end_frac` | `0.70` | Fraction of facet period where the active scanline ends (`scanline_ticks = 42 000`) |
| `pd_debounce_us` | `20.0` µs | Photodiode trigger & re-arm debounce duration (`480` ticks at `24 MHz`) |
| `spinup_time` | `1.5` s | Spin-up wait before enabling the sync laser |
| `stable_time` | `1.125` s | Initial photodiode lock timeout |
| `hfosc_div` | `1` | Internal iCE40 `SB_HFOSC` divider (`24 MHz` clock → `120 000` ticks/facet) |
| `samplegridsize` | `0.01` mm | Vector-to-raster fallback pitch (`.ps` / rasterized `.svg`) and debug-plot resolution (`10 µm` → `bitsinscanline = 560`, padded to a multiple of 8). True bitmaps use their native `pixelsize` (or embedded DPI) directly. |
| `single_line` | `False` | Repeat the buffered line(s) continuously when FIFO hits empty |
| `singlefacet` | `False` | Expose on only one facet per revolution |

> **Tick-native timing, native bitmaps & fallback raster grid:** `VectorExposure` (`PrismCurve.u_to_tick()`), direct bitmap slicing (`RasterExposure.slice_bitmap()`), and `Laserhead` laser modulation and step generation all operate directly in `24 MHz` FPGA clock ticks (`41.7 ns` resolution) across `scanline_ticks = round(facet_ticks × (end_frac − start_frac))`. True bitmaps (`.png`, `.jpg`, etc.) are never resized onto `samplegridsize`; their scanlines intersect native `pixelsize` boundaries and emit `(tick, intensity)` RLE events on transitions (binary by default, or opt-in `grayscale=True` / `gray_levels = 32` / `intensity_lut`). `samplegridsize` (`10 µm` → `560` samples/line) applies only to the vector-to-raster fallback (`.ps`/`.eps`, `.svg` with `mode="raster"` or `grayscale=True`) and PC debug reconstruction (`plotter.py` / `readbin()`).

---

# Laserhead Stability & Optical Calibration

## 1. Statistical Laserhead Stability Test (`test_laserhead`)

Implemented in [src/hexastorm/fpga_host/micropython.py](src/hexastorm/fpga_host/micropython.py), backed by [src/hexastorm/lasers.py](src/hexastorm/lasers.py) and [src/hexastorm/fpga_host/interface.py](src/hexastorm/fpga_host/interface.py).

On every valid synchronized photodiode pulse in `Laserhead` (`WAIT_STABLE`), the FPGA latches `facet_period_ticks = Cat(facetcnt, tickcounter)` and exposes `[ticks_per_facet, facet_id]` via `Spi.Commands.debug` (`BaseHost.read_facet_ticks_and_id()`). `ESP32Host.measure_facet_period_ms(samples=200)` converts measured ticks to milliseconds and sleeps for a randomized interval (`dt_facet_ms + 1 .. 2 × dt_facet_ms` ms, plus `0 .. 4 × dt_facet_ms` ms every 25 samples) so every read captures a fresh facet measurement without aliasing with prism rotation.

`ESP32Host.test_laserhead(shift=0, ref_facet_means=None)` maps raw hardware facet indices to logical facets via `(f_id + shift) % num_facets` and decomposes timing variance into static facet cut/geometry differences, motor speed jitter, and photodiode/timer noise across two stages:

1. **Part 1 — Global RPM & Motor Stability:** Computes global mean period against the nominal period `60 / (rpm × facets / 1000)` (`5.0 ms` at `3000 RPM`, `4` facets) and derives `measured_rpm`. Fails immediately if `global_deviation_perc > 10.0%`.
2. **Part 2 — Per-Facet Process Capability (`Cpk`) & 3-Sigma Jitter:** Requires `≥ 50` samples per facet (targets `N = 200`, which under Chi-Square assumptions limits false alarm rate `α < 0.001%` and missed defect rate `β < 0.003%`) and evaluates each facet against `jitter_exp_perc = 0.2%`:

```text
std_dev_perc       = (std / mean) × 100
three_sigma_jitter = 3 × std_dev_perc
Cpk                = jitter_exp_perc / (3 × std_dev_perc)
Pass criterion     : Cpk ≥ 1.0  (three_sigma_jitter ≤ 0.2% per facet → 99.73% of spots in spec)
```

* **Zero-camera feed-forward option (`PlatformConfig.compute_photodiode_scan_corrections()`):** Converts per-facet timing deviations `delta_t = period_i − median(periods)` directly into preliminary along-scan shifts (`scan_offset_mm = k_scale × delta_t`), using `k_scale = v_scan(0) × center_frac = 3.10 mm/ms × 0.525 = 1.628 mm/ms` (where `center_frac = (start_frac + end_frac) / 2`). This mitigates dynamic rotational timing drift before/without optical imaging, while static mechanical facet wedge and orthogonal offsets are measured optically below.

## 2. Closed-Loop Dot-Pattern Calibration & `LaserStackSimulator`

Implemented in [src/hexastorm/calibration/analyzer.py](src/hexastorm/calibration/analyzer.py), [src/hexastorm/calibration/stack.py](src/hexastorm/calibration/stack.py), and [tests/camera/test_optical.py](tests/camera/test_optical.py).

Because optical bench verification uses a stationary camera sensor (no moving stage or photoresist needed), calibration and 2D validation follow a four-step closed loop:

1. **Camera Visibility Scan (`DynamicTests.scan_visibility`):** Sweeps a single lit position across the `10 µm` software scanline grid (`bitsinscanline = 560`, default stride `10`), captures frames with `picture_line`, runs `get_dots()`, and records which grid indices fall inside the camera viewport in `output/calibration/scan_visibility.json`.
2. **1D Dot-Pattern Measurement (`DynamicTests.baseline_calibration_cycle` + `analyzer.py`):**
   * Bypasses the slicer via `direct_pattern()` (`[1] + [0]×39` repeated across `bitsinscanline`) and captures `facet{f}_iter{it}.jpg` per facet across `iterations = 10` runs alongside live photodiode facet means (`host.cur_facet_means`).
   * `get_dots()` applies Otsu thresholding (minimum floor `50`), dynamic morphological closing (`min_diameter_um` scaled by camera pixel pitch), contour area filtering (`10–200 µm` default for baseline 1D lines), moment centroids, and `cv.fitEllipse` for major/minor axes and eccentricity.
   * `compute_angle()` + `rotate_dots()` fit the dot line slope per facet, take the global median tilt angle across facets, and rectify coordinates horizontally.
   * `calculate_facet_shifts()` pairs corresponding dots within `max_shift_um = 200.0 µm` and constructs a **Virtual Reference Facet** using the per-dot **spatial median** across all 4 facets (so one severely misaligned facet cannot skew the reference). Each facet's `Scan_shift_um` and `Orth_shift_um` are computed against this reference.
   * `update_statistical_history()` aggregates repeatability metrics (`mean`, `std`, `ptp`, `rotation_angle`, `facet_period_ms`, `spot_size_um`, `eccentricity`) and appends millimeter corrections (`scan`, `orth`) to `output/calibration/calibration_history.json`.
3. **Optimized / Corrected Exposure (`PlatformConfig.get_optical_params(correction=True)`):** Reads the latest `calibration_history.json` entry (`f{i}_scan`, `f{i}_orth` in mm) and applies those offsets in `ExposureEngine` / `VectorExposure` / `RasterExposure` / `CameraCalibrationGen` when generating exposure streams.
4. **Closed-Loop 2D Verification (`DynamicTests.pattern_error` + `LaserStackSimulator`):**
   * Uses `CameraCalibrationGen.generate_dot_grid_test()` (sized to the visible pixel bounds from `scan_visibility.json`) to generate the same 2D dot grid twice: uncorrected (`correction=False`) and corrected (`correction=True`, `exposures=4`).
   * Reads raw `24 MHz` scanline events via `cam_pat.engine.readbin_events()` and, for each facet, projects every active scanline row (`row in range(facet, lines, tot_facets)`) statically onto the camera with `picture_events()` (calling `host.write_events()`, avoiding bitmap re-quantization). Each capture is passed to `LaserStackSimulator.add_exposures(img, num_steps=row)`, which Otsu-binarizes the frame, rotates by the calibrated `mean_angle_deg` (or shifts along the angle vector when `rotate=False`), translates orthogonally by `num_steps × step_px` (where `step_px = (1 / steps_mm) × 1e3 / (px_size_um × exposures)`), and merges onto the virtual 2D canvas via `np.maximum`.
   * Running `calibration(..., compute_rotation=False)` on the 4 stacked facet images compares uncorrected vs. corrected 2D errors and asserts `max_err_after ≤ 30.0 µm` (`TOLERANCE_UM`) (and improvement if `max_err_before > 30.0 µm`). (`DynamicTests.line_error` provides a faster 1D vertical-line before/after check, also using `readbin_events()` + `picture_events()`.)

---

# Camera Setup & Configuration

On the PC running OpenCV calibration analysis (`tests/camera/`, `src/hexastorm/calibration/`), install the camera extras:
```console
uv sync --extra camera
```

## 1. Onboard ESP32-S3 + OV2640 (Target Setup)

The project is transitioning from the external Raspberry Pi bench camera to an **OV2640** camera module wired directly to the **ESP32-S3** host. Hardware specs, I2C address, JPEG capture defaults, and DVP GPIO pins are centralized in `Camera.OV2640` and `PlatformConfig.esp32_cfg["camera"]` ([src/hexastorm/config.py](src/hexastorm/config.py)):

| Setting / Signal | Value (`Camera.OV2640`) | Notes |
| :--- | :--- | :--- |
| Resolution & pixel pitch | `1600×1200` (`FrameSize.UXGA`), `2.2 µm` | `4.4 µm` in SVGA mode |
| Capture format | `PixelFormat.JPEG`, `jpeg_quality = 85` | `exposure_ms = 200`, `fb_count = 1`, `GrabMode.LATEST` |
| I2C control bus | Address `0x30` on `SCL = GPIO 17`, `SDA = GPIO 18` | Shared `100 kHz` I2C bus with digipot (`0x28`) |
| Parallel DVP data (`D0..D7`) | GPIOs `12, 14, 21, 13, 11, 10, 9, 8` | 8-bit parallel interface |
| Sync & clock pins | `PCLK = 38`, `VSYNC = 41`, `HREF = 40`, `XCLK = 39` | `XCLK = 10 MHz` |
| Power & reset | `RESET = GPIO 42`, `PWDN = -1` | Keep `RESET` high when accessing the I2C bus |

## 2. Legacy External Bench: Raspberry Pi + Arducam OV2311 (V4L2)

The existing `tests/camera/` verification scripts support the legacy external **Arducam Global Shutter (OV2311, `1600×1300`, `3.0 µm` pixel)** on Raspberry Pi OS / Debian 12 or 13 (**Bookworm/Trixie**) 64-bit using Linux kernel V4L2 + Media Controller drivers (`Camera.OV2311`).

### 2.1 Boot Configuration

Add the following Device Tree overlays to `/boot/firmware/config.txt` to load `ov2311` manually via V4L2/Media Controller:

```ini
# Disable auto-detection to ensure manual control via V4L2
camera_auto_detect=0

# Enable I2C bus for camera/display
dtparam=i2c_vc=on
dtparam=i2c_vc_baudrate=100000

# Kernel DRM driver for video/graphics
dtoverlay=vc4-kms-v3d
max_framebuffers=2

[all]
# Hardware-specific overlay for OV2311
# cam1 forces the port on CM4/Pi4; media-controller=1 is essential for RAW access
dtoverlay=ov2311,cam1,media-controller=1
gpu_mem=300
arm_64bit=1
arm_boost=1

# Disable Bluetooth to prevent WiFi interference and terminal latency
dtoverlay=disable-bt
```

### 2.2 Automatic Initialization (Systemd)

Because the OV2311 media pipeline (pixel format and resolution) resets on reboot, initialize it via a script and systemd service.

#### Initialization Script

Create `/usr/local/bin/camera-init.sh`:

```bash
#!/bin/bash
# Find the correct media node (typically /dev/media3 or 4 on Pi 4)
MEDIA_NODE=$(media-ctl -d /dev/media* -p | grep -l "ov2311" | head -n 1)

# Configure the hardware pipeline to 8-bit RAW (GREY)
media-ctl -d "$MEDIA_NODE" --set-v4l2 '"ov2311 10-0060":0[fmt:Y8_1X8/1600x1300]'
v4l2-ctl -d /dev/video0 --set-fmt-video=width=1600,height=1300,pixelformat=GREY

# Optimize system stability
/sbin/iwconfig wlan0 power off  # Disable WiFi Power Management to prevent SSH freezes
v4l2-ctl -d /dev/v4l-subdev0 -c exposure=500  # Set a default baseline exposure
```

Make the script executable: `sudo chmod +x /usr/local/bin/camera-init.sh`.

#### Systemd Service

Create `/etc/systemd/system/camera-init.service`:

```ini
[Unit]
Description=Initialize Arducam OV2311 V4L2 Pipeline
After=multi-user.target

[Service]
Type=oneshot
ExecStart=/usr/local/bin/camera-init.sh
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

Enable the service: `sudo systemctl enable --now camera-init.service`.
