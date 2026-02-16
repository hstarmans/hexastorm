# Dev notes

The camera is a major dependency. My version of arducam does not support the latest raspberry pi OS.
As a result, I fixed numpy to 1.19.5 and python to 3.9.2.

## Precommit
Install Git pre-commit hooks based on the .pre-commit-config.yaml file.
```pre-commit install```

## Jupyter lab

Jupyter lab only works with git extension if key is added to ssh agent.
```ssh-ad id_ed25519```
You can use the following script to launch jupyter lab.
```
#!/bin/bash

env -C ~/projects/hexastorm/ poetry run jupyter lab --no-browser --ip "*"
```

## Notes removed from installation
On Raspbian, install libatlas so latest Numpy, etc. can be installed via pip.
```console
sudo apt update
sudo apt install libatlas3-base
```
Install [luna](https://github.com/greatscottgadgets/luna) and checkout at f54de01.
So after git cloning, run ```git checkout f54de01```.
Priop to yowasp apio was used but this result in a suboptimal binary.
```console
pip3 install yowasp-yosys
pip3 install yowasp-nextpnr-ice40-all
```
If yowasp is used, add in  ```~/.bashrc```
```
## add python files to path
export PATH="/home/ubuntu/.local/bin:$PATH"
## these lines only needed for ubuntu core
export YOSYS="yowasp-yosys"
export ICEPACK="yowasp-icepack"
export NEXTPNR_ICE40="yowasp-nextpnr-ice40"
```
Run  ```source ~/.bashrc``` afterwards.
You can enable wifi using [link](https://github.com/sraodev/Raspberry-Pi-Headless-Setup-via-Network-Manager).
Using the raspberry pi installer you can automatically fix wifi and it will copy password.

If the behaviour of the FPGA is simulated, signal traces for [GTKWave](http://gtkwave.sourceforge.net/) are generated if the following flag is set.
```console
export GENERATE_VCDS=1
```
After rebooting, you can check the configuration via
```console
ls /dev/spi*
sudo vcdbg log msg
```
Usefull links are [1](http://terminal28.blogspot.com/2016/05/enabling-spi1-on-raspberry-pi-bzero23.html) and [2](https://bootlin.com/blog/enabling-new-hardware-on-raspberry-pi-with-device-tree-overlays/).  

The slicer relies on numba for acceleration. Latest is llvm-12 but pip3 only supports 10 now.
```console
sudo apt-get install llvm-10
LLVM_CONFIG=/usr/lib/llvm-10/bin/llvm-config pip3 install llvmlite
pip3 install numba
```
The location of llvm-config can be found with
```console
find / -name llvm-config
```

## Notes removed from Camera
The operation of the laser scanner can be verified with a camera.
Two camera's have been tried; [uEye](https://en.ids-imaging.com/) 2240 monochrome camera and [Arducam Global shutter](https://www.arducam.com/products/camera-breakout-board/global-shutter-camera/) which used the OV2311 chip. I use an old version UC-621 which is not compatible with the latest version of raspberry and drivers.
The images of the cameras are analyzed with [OpenCV](https://opencv.org/).
```console
sudo apt install -y libopenjp2-7 libilmbase-dev libopenexr-dev libgstreamer1.0-dev ffmpeg
pip3 install opencv-python
```
Camera must be enabled via raspi-config. Raspi-config states that this will no longer be supported in future versions and denotes camera as legacy.
I had to leave 'i2c-dev' in '/etc/modules-load.d/modules.conf' for i2c to load.

## Arducam
Laser spots are measured using the arducam UC-621. There is a binary available for 64 bit, this
binary only works on the GNU/Linux 11 (bullseye). No sources are available and it cannot be recompiled. There is a new version
of this camera by arducam but its driver cannot be used by UC-621. The new version uses the libcam driver. My camera is not
libcam compatabile. For more information see https://github.com/ArduCAM/MIPI_Camera.
Before you install my arducampython wrapper. I have the following notes.
The following lines need to be in the /boot/config.txt for the arducam to work. If all is well you should see the camera via 
i2cdetect -y 10;
```
# I2C for laserdriver and camera
dtparam=i2c_arm=on
dtparam=i2c_vc=on
# SPI
dtoverlay=spi0-1cs,cs0_pin=18
dtoverlay=spi1-1cs,cs0_pin=7
# camera
dtoverlay=vc4-fkms-v3d
start_x=1
gpu_mem=300
```
There should not be dtparam=spi=on, somewhere. This would enable two chip selects for SPI0 and 
creates a conflict with the pin select of SPI1. 

#### uEye camera
Disadvantages; uEye is more expensive, drivers require an account and there is no good Python driver.  
Advantages; product is more mature.  
On uEye website select uEye 2240 monochrome. Download and install the driver for
linux, arm v7 (Raspberry pi platform). A Python library for the camera is available in the source code.
My version can be installed via [uEyeCamera](https://github.com/hstarmans/ueyecamera).

# Brief Description

The controller sends over a command with a word to the FPGA which stores it in SRAM.
The command is 8 bits long and the word 64 bits. Only for write commands, word is not empty.
If the memory is full, the FPGA send this back to the host. 
The instructions are parsed from the the SRAM if execution is enabled.

## Commands
The following commands are possible;
| command  | reply                                                    |
| -------- | -------------------------------------------------------- |
| POSITION | get position of all motors                               |
| READ     | get state of the peripheral and settings of certain pins |
| START    | enable execution of instructions stored in SRAM          |
| STOP     | halt execution of instructions stored in SRAM            |
| WRITE    | sent over an instruction and store it in SRAM            |


## Write
A write commmand is followed by an instruction which is placed in the SRAM.
If the dispatcher is enabled, these instructions are carried out unless an error is raised.
A word can often not store all information for an instruction. So an instruction 
consists out of multiple commands and words in series.
If prior to the sequence, the memory is already full or there is a parsing error, a status word is sent back.
If the reply is zero, the peripheral is operating normally. The information to be sent over is indicated for
each instruction

## Parameters
The following parameters describe the system.  
| parameter    | description                                                      |
| ------------ | ---------------------------------------------------------------- |
| RPM          | revolutions per minute of the rotor                              |
| Start%       | fraction of period where scanline starts                         |
| End%         | fraction of period where scanline stops                          |
| SPINUP_TIME  | seconds system waits for the rotor to stabilize speed            |
| STABLE_TIME  | seconds system tries to determine position laser with photodiode |
| FACETS       | number of polygon facets                                         |
| DIRECTION    | exposure direction, i.e. forward or backward                     |
| SINGLE_LINE  | system exposes fixed pattern, i.e. line                          |
| SINGLE_FACET | only one of the facets is used                                   |
  
Using the above, the code determines the number of bits in a scanline. Via a serial port interface the user can push data to the scanner.
A line is preceded with a command which can be SCAN or STOP. The data is stored on the chip in block ram. 
Once turned on, the scanner reads out this memory via First In First Out (FIFO).
  
### Motion

A robot updates its position multiple times during a move. In practice it is impossible to sent over each position indivually due to data transfer limits.
The controller solves this via interpolation.  The starting and end-conditions, i.e. boundary conditions, are given for a move.
Possible boundary conditions are not only position but can be acceleration, jerk or number of steps in a move. It depends on the mathematical formulation chosen.
I extensively looked at two mathematical solutions; [splines](https://en.wikipedia.org/wiki/Spline_(mathematics)) and [Bezier curves](https://en.wikipedia.org/wiki/B%C3%A9zier_curve).
Other options are B-splines and NURBS (Non-Uniform Rational B-splines) see [article](https://zero.sci-hub.se/2496/cb390d406cc077ef156deb76b34099af/desantiago-perez2013.pdf#lb0030).
Most programs like Cura and slicer store the final instructions as [G-code](https://en.wikipedia.org/wiki/G-code).
There is no link between g-code and my interpretation yet.

### Splines
| data        | number of bytes | description                                                              |
| ----------- | --------------- | ------------------------------------------------------------------------ |
| INSTRUCTION | 1               | type of instructions, here move instruction                              |
| TICKS       | 7               | number of ticks in a move, cannot be larger than TICKS_MOVE, i.e. 10_000 |
| C00         | 8               | motor 0, coeff 0                                                         |
| C01         | 8               | motor 0, coeff 1                                                         |
| C02         | 8               | motor 0, coeff 2                                                         |

The motor follows the path, C00 * t + C01 * t^2 + C02 * t^3. The default motor sampling frequency is 1 MHz.
The coefficients can be interpreted as; velocity, acceleration and jerk. These are slightly different.
In the formula x = v*t + 1/2*a*t^2 + 1/3*1/2*b*t^3; v, a, b, x are the velocity
acceleration, jerk and position respectively.
The trajectory of a motor is divided in multiple segments where a segment length has a maximum of 10_000 ticks. 
If is longer, it is repeated. If it is shorter, this is communicated by setting ticks to lower than 10_000.
If multiple motors are used; TICKS, C00, C01, C02 are repeated. Step speed must be lower than 1/2 oscillator speed (Nyquist criterion).
For a [typical stepper motor](https://blog.prusaprinters.org/calculator_3416/) with 400 steps per mm,
max speed is 3.125 m/s with an oscillator frequency of 1 MHz.

### Bezier Curves

I made a sketch for an implementation in test/old/dsp/casteljau.py

### Pin instruction
| data        | number of bytes | description                                                   |
| ----------- | --------------- | ------------------------------------------------------------- |
| INSTRUCTION | 1               | type of instructions, here set pin instruction                |
| PINS        | 7               | Last byte set pins. The last bits set polygon, laser0, laser1 |

A user can read but not write directly to pins. This ensures that the host
can establish precedence between instructions.

### Laserline instruction
| data         | number of bits | description                                       |
| ------------ | -------------- | ------------------------------------------------- |
| INSTRUCTION  | 8              | type of instructions, here start or stop scanline |
| DIRECTION    | 1              | scanning direction                                |
| TICKSPERSTEP | 55             | ticks per half period step                        |
| DATA         | 64             | information for lasers in chunks of 8 bytes       |

A user can read but not write directly to pins. This ensures that the host
can establish precedence between instructions.

## Alignment procedure

You start by printing several patterns with a different dosage.
The correct dosage and the actual size of a lane is determined.
You then try several offsets between lanes to find the optimum.
The backlash is ignored for the moment. 

## Detailed description
Look at the test folders and individually tests on how to use the code. The whole scanhead can be simulated virtually. 
As such, a scanner is not needed.

## Limitations 
System is for writing to, i.e. exposing, a substrate. Reading should also be possible to enable optical coherence tomagraphy.    
System has no link for LIDAR measurements, circuit can be found [here](https://hackaday.io/project/163501-open-source-lidar-unruly).    
The FPGA controls all the stepper motors. At the moment it is not possible to read instruction from a GCODE file.  
Add maximum-length linear-feedback shift register sequence and CRC check.  


<!-- 
TODO:
  add docs
 -->
