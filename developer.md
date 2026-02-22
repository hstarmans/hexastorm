# Dev Notes

## Camera Setup

The operation of the laser scanner can be verified with a camera. We use the [Arducam Global shutter](https://www.arducam.com/products/camera-breakout-board/global-shutter-camera/) which uses the OV2311 chip. Note that the older UC-621 version is not compatible with the latest Raspberry Pi versions and drivers.

The camera must be enabled via `raspi-config`. Note that `raspi-config` states this will no longer be supported in future versions and denotes the camera as legacy. You may need to leave `i2c-dev` in `/etc/modules-load.d/modules.conf` for the I2C to load properly.

## Arducam Configuration

Laser spots are measured using the Arducam UC-621. There is a binary available for 64-bit systems, but this binary only works on GNU/Linux 11 (Bullseye). No sources are available, so it cannot be recompiled. Arducam has released a newer version of this camera that uses the `libcam` driver, but the UC-621 is not `libcam` compatible. For more information, see the [ArduCAM MIPI_Camera GitHub repository](https://github.com/ArduCAM/MIPI_Camera).

Before installing the Python wrapper, ensure the following lines are in your `/boot/config.txt` for the Arducam to work. If successful, the camera should be visible via `i2cdetect -y 10`:

```ini
dtparam=i2c_arm=on
dtoverlay=spi0-1cs,cs0_pin=18
dtoverlay=spi1-1cs,cs0_pin=7
dtparam=audio=on
start_x=1
display_auto_detect=1
dtoverlay=vc4-kms-v3d
max_framebuffers=2
dtoverlay=gpio-poweroff,gpiopin=17,active_low
arm_64bit=1
disable_overscan=1

[cm4]
otg_mode=1

[pi4]
arm_boost=1

[all]
dtparam=i2c_vc=on
gpu_mem=300
dtparam=i2c_arm_baudrate=400000
```

**Important:** There should not be `dtparam=spi=on` anywhere else in the file. This would enable two chip selects for SPI0 and create a conflict with the pin select of SPI1.

The correct Python dependencies for Bullseye can be obtained using Conda. Use `uv` to install the remaining packages in your Conda environment:

```bash
wget "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-aarch64.sh"
bash Miniforge3-Linux-aarch64.sh
source .bashrc
conda create -n hexastorm python=3.12 numba "numpy>=2.0.0"
conda install "opencv>=4.10.0"
conda activate hexastorm
uv pip install --system -e ".[camera,desktop]"
```

---

# Brief Description

The controller sends a command with a word to the FPGA, which stores it in SRAM. The command is 8 bits long, and the word is 64 bits. The word is only non-empty for write commands. If the memory is full, the FPGA sends a notification back to the host. The instructions are parsed from the SRAM if execution is enabled.

## Commands

The following commands are supported:

| Command | Reply |
| --- | --- |
| **POSITION** | Get the position of all motors |
| **READ** | Get the state of the peripheral and settings of certain pins |
| **START** | Enable execution of instructions stored in SRAM |
| **STOP** | Halt execution of instructions stored in SRAM |
| **WRITE** | Send over an instruction and store it in SRAM |

## Write Instruction

A write command is followed by an instruction placed into the SRAM. If the dispatcher is enabled, these instructions are executed unless an error is raised. Because a single 64-bit word often cannot store all the necessary information, an instruction usually consists of multiple commands and words in series.

If the memory is full prior to the sequence, or if there is a parsing error, a status word is sent back. If the reply is zero, the peripheral is operating normally.

## Parameters

The following parameters describe the system's operational state:

| Parameter | Description |
| --- | --- |
| **RPM** | Revolutions per minute of the rotor |
| **Start%** | Fraction of the period where the scanline starts |
| **End%** | Fraction of the period where the scanline stops |
| **SPINUP_TIME** | Seconds the system waits for the rotor to stabilize speed |
| **STABLE_TIME** | Seconds the system tries to determine the laser position with the photodiode |
| **FACETS** | Number of polygon facets |
| **DIRECTION** | Exposure direction (forward or backward) |
| **SINGLE_LINE** | System exposes a fixed pattern (a single line) |
| **SINGLE_FACET** | Only one of the facets is used |

Using the parameters above, the code determines the number of bits in a scanline. Via a serial port interface, the user can push data to the scanner. A line is preceded with a command, which can be `SCAN` or `STOP`. The data is stored on the chip in block RAM. Once turned on, the scanner reads out this memory via a First-In-First-Out (FIFO) buffer.

---

# Motion Control

A robot updates its position multiple times during a move. In practice, it is impossible to send over each position individually due to data transfer limits. The controller solves this via interpolation. Boundary conditions (starting and end conditions) are provided for a move.

These boundary conditions aren't limited to position; they can include acceleration, jerk, or the number of steps in a move, depending on the mathematical formulation chosen. Two common mathematical solutions are Splines and Bezier curves.  Other options include B-splines and NURBS (Non-Uniform Rational B-splines).

While programs like Cura store final instructions as G-code, there is currently no direct link between standard G-code and this system's interpretation.

## Splines

| Data | Bytes | Description |
| --- | --- | --- |
| **INSTRUCTION** | 1 | Type of instruction (Move instruction) |
| **TICKS** | 7 | Number of ticks in a move (Max: TICKS_MOVE = 10_000) |
| **C00** | 8 | Motor 0, Coefficient 0 |
| **C01** | 8 | Motor 0, Coefficient 1 |
| **C02** | 8 | Motor 0, Coefficient 2 |

The motor follows the mathematical path:


The default motor sampling frequency is 1 MHz. The coefficients can be interpreted as velocity, acceleration, and jerk, corresponding to the standard kinematics equation:

*(where  is velocity,  is acceleration,  is jerk, and  is position).*

The trajectory of a motor is divided into multiple segments. A segment length has a maximum of 10_000 ticks. If a move is longer, it is repeated; if shorter, it is communicated by setting the ticks lower than 10_000. For multiple motors, the `TICKS`, `C00`, `C01`, and `C02` data blocks are repeated.

*Note: Step speed must be lower than 1/2 of the oscillator speed to satisfy the Nyquist criterion. For a typical stepper motor with 400 steps per mm, the maximum speed is 3.125 m/s with an oscillator frequency of 1 MHz.*

## Pin Instruction

| Data | Bytes | Description |
| --- | --- | --- |
| **INSTRUCTION** | 1 | Type of instruction (Set pin instruction) |
| **PINS** | 7 | The last byte sets the pins. The last bits set polygon, laser0, laser1 |

A user can read from, but not write directly to, the pins. This restriction ensures that the host system can properly establish precedence between instructions.

## Laserline Instruction

| Data | Bits | Description |
| --- | --- | --- |
| **INSTRUCTION** | 8 | Type of instruction (Start or stop scanline) |
| **DIRECTION** | 1 | Scanning direction |
| **TICKSPERSTEP** | 55 | Ticks per half-period step |
| **DATA** | 64 | Information for lasers in chunks of 8 bytes |

Similar to pin instructions, users can read but not directly write to the hardware pins, maintaining instruction precedence from the host.
