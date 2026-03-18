# Dev Notes

# Camera Setup & Configuration

The operation of the laser scanner is verified using an **Arducam Global Shutter** camera (OV2311 chip). This setup utilizes the standard Linux kernel drivers (V4L2) 
instead of libcam.

## System Requirements
* **Hardware:** Arducam OV2311 (Global Shutter).
* **OS:** Raspberry Pi OS / Debian 12 or 13 (**Bookworm/Trixie**) 64-bit.
* **Driver Model:** V4L2 (Video4Linux2) with Media Controller support.

## 1. Boot Configuration
The camera is configured via Device Tree overlays in `/boot/firmware/config.txt`. This forces the Pi to use the correct I2C bus and load the `ov2311` driver without relying on the deprecated `start_x` legacy stack.

Add the following lines to `/boot/firmware/config.txt`:

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
---

Je hebt gelijk, de weergave versprong daar doordat er een codeblok *binnen* een codeblok stond. Dat vindt de interface niet leuk.

Hier is de tekst nogmaals, maar nu in een "raw" format zonder nesten, zodat je het veilig kunt kopiëren:

---

# Camera Setup & Configuration

The operation of the laser scanner is verified using an **Arducam Global Shutter** camera (OV2311 chip). Unlike legacy implementations, this setup utilizes the standard Linux kernel drivers (V4L2) for maximum stability, low latency, and full resolution support on modern Raspberry Pi systems.

## System Requirements

* **Hardware:** Arducam OV2311 (Global Shutter).
* **OS:** Raspberry Pi OS / Debian 12 or 13 (**Bookworm/Trixie**) 64-bit.
* **Driver Model:** V4L2 (Video4Linux2) with Media Controller support.

## 1. Boot Configuration

The camera is configured via Device Tree overlays in `/boot/firmware/config.txt`. This forces the Pi to use the correct I2C bus and load the `ov2311` driver without relying on the deprecated `start_x` legacy stack.

Add the following lines to `/boot/firmware/config.txt`:

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

## 2. Automatic Initialization (Systemd)

Because the OV2311 is an industrial sensor, the "pipeline" (pixel format and resolution) must be initialized at every boot. This is handled by a bash script and a managed systemd service.

### Initialization Script

Create the script at `/usr/local/bin/camera-init.sh`:

```bash
#!/bin/bash
# Find the correct media node (typically /dev/media3 or 4 on Pi 4)
MEDIA_NODE=$(media-ctl -d /dev/media* -p | grep -l "ov2311" | head -n 1)

# Configure the hardware pipeline to 8-bit RAW (GREY)
media-ctl -d $MEDIA_NODE --set-v4l2 '"ov2311 10-0060":0[fmt:Y8_1X8/1600x1300]'
v4l2-ctl -d /dev/video0 --set-fmt-video=width=1600,height=1300,pixelformat=GREY

# Optimize system stability
/sbin/iwconfig wlan0 power off  # Disable WiFi Power Management to prevent SSH freezes
v4l2-ctl -d /dev/v4l-subdev0 -c exposure=500  # Set a default baseline exposure

```

*Ensure the script is executable: `sudo chmod +x /usr/local/bin/camera-init.sh*`

### Systemd Service

Create the service unit at `/etc/systemd/system/camera-init.service`:

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

*Enable the service: `sudo systemctl enable --now camera-init.service*`



# FPGA description

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
