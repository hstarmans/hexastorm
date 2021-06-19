# Laserscanner [![Documentation Status](https://readthedocs.org/projects/luna/badge/?version=latest)](https://hexastorm.readthedocs.io/en/latest/?badge=latest)
Implementation of a laserscanner on a FPGA. In a high-speed polygon scanner system, the laser is deflected by a rotating prism or reflective mirror. 
The position of the laser is determined via a sensor such as a photodiode.  
<img src="https://cdn.hackaday.io/images/7106161566426847098.jpg" align="center" height="300"/>  
Code is tested on the system shown above, branded as [Hexastorm](https://www.hexastorm.com). 
The bill of materials (BOM) and links to FreeCad and PCB designs can be found on
[Hackaday](https://hackaday.io/project/21933-open-hardware-fast-high-resolution-laser).
The code took most inspiration from [LDGraphy](https://github.com/hzeller/ldgraphy).  
A video of the scanhead exposing with latest code can be seen below;  
[![video in action not showing](https://img.youtube.com/vi/KQgnZkochu4/0.jpg)](http://www.youtube.com/watch?v=KQgnZkochu4 "Moving laserhead").

The alignment procedure is shown in the following video.

[![Alignment procedure image not showing](http://img.youtube.com/vi/Ri6DAneEzw4/0.jpg)](http://www.youtube.com/watch?v=Ri6DAneEzw4 "Alignment procedure")

## Install Notes
The code works on Raspberry Pi 3B and beyond. The SD-card should be at least 8 GB, ideally 16 gb.
Both Raspbian and Ubuntu can be used. Raspbian is not yet available at 64 bit.
Besides 64 bit, Ubuntu has the advantage that latest toolchain for Yosys is easier to install.
The arducam, used in the alignment, does not work at 64 bit.
On Raspbian, install libatlas so latest Numpy, etc. can be installed via pip.
```console
sudo apt update
sudo apt install libatlas3-base
```
Install luna and checkout at f54de01. Code after this date does not work yet.
Install required libraries
```console
pip3 install -r requirements.txt
```
Install Hexastorm in develop mode so you can edit.
```console
python3 setup.py develop --user
```
On 32 bits raspbian, install ice40 and yosys. These are outdated but work. For the latest, you need to build from source.
```console
apio install yosys
apio install ice40
```
On 64-bit use
```console
pip3 install yowasp-yosys
pip3 install yowasp-nextpnr-ice40-8k
```
Install icezprog, on ubuntu you need to add ```-lcrypt -lm``` to makefile.
```console
git clone https://github.com/cliffordwolf/icotools
cd ~/icotools/examples/icezero
make icezprog
mv icezprog ~/.local/bin
```
In the ```~/.bashrc``` for Raspbian add 
```
export PATH=/home/pi/.local/bin:$PATH
export PATH=/home/pi/.apio/packages/toolchain-yosys/bin:$PATH
export PATH=/home/pi/.apio/packages/toolchain-ice40/bin:$PATH
``` 
for Ubuntu add
```
## add python files to path
export PATH="/home/ubuntu/.local/bin:$PATH"
## these lines only needed for ubuntu core
export YOSYS="yowasp-yosys"
export ICEPACK="yowasp-icepack"
export NEXTPNR_ICE40="yowasp-nextpnr-ice40"
```
You can enable wifi using [link](https://github.com/sraodev/Raspberry-Pi-Headless-Setup-via-Network-Manager)

The slicer relies on numba for acceleration
```
# latest is 12 but pip3 only supports 10 for now
sudo apt-get install llvm-10
# if you can't locate llvm-config use
# find / -name llvm-config
LLVM_CONFIG=/usr/lib/llvm-10/bin/llvm-config pip3 install llvmlite
pip3 install numba
```

## FPGA Debugging
Signal traces for GTKWave can be generated via;
```
export GENERATE_VCDS=1
```

## Stepper drivers
Install the python wrapper for TMC stepper [drivers](https://github.com/hstarmans/TMCStepper).

### OpenCV 
For image operations, opencv is required.
```console
pip3 install opencv-python
```
Also install the following dependencies
```console
sudo apt install -y libopenjp2-7 libilmbase-dev libopenexr-dev libgstreamer1.0-dev ffmpeg
```

### Camera
Two camera's have been tried; uEye camera and Arducam Global shutter ov2311.
Currently, the ov2311 chip is used.

#### uEye camera
Disadvantages; the uEye is more expensive, drivers require an account and there is no good Python driver.  
Advantages; the product is more mature.  
On Ueye website select Ueye 2240 monochrome. Download and install the driver for
linux, arm v7, as this is the raspberry pi platform.  A python library for the camera is available in the source code.
My version can be installed via [uEyeCamera](https://github.com/hstarmans/ueyecamera).

#### Arducam
Install my version of the Python libary [ArducamPython](https://github.com/hstarmans/Arducampython).


### Config
Raspberry pi uses ```/boot/config.txt``` and ubuntu uses ```/boot/firmware/usercnf.txt.```
The following lines need to be available;


In the current board, the SPI1-1 select pin is not routed to the correct pin on the Raspberry.
In /boot/config.txt ensure you have the following
```
# I2C for laserdriver and camera
i2c_arm=on
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
create a conflict with the pin select of SPI1. You can check the configuration via
```console
ls /dev/spi*
sudo vcdbg log msg
```
Usefull links are [1](http://terminal28.blogspot.com/2016/05/enabling-spi1-on-raspberry-pi-bzero23.html) and [2](https://bootlin.com/blog/enabling-new-hardware-on-raspberry-pi-with-device-tree-overlays/).

### I2C
In the current scanhead, I2C is used to set the power of the laser via a digipot.
I2C can be enabled on the Raspberry Pi as [follows](https://pimylifeup.com/raspberry-pi-i2c/).
```console
i2cdetect -y 1
```
This will produce output, here 28 is the address of the I2C device.
```console
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- 28 -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
70: -- -- -- -- -- -- -- --
```
## Motion
Splines, Bezier, B-splines, and NURBS (Non-Uniform Rational B-splines) curves are the common parametric techniques 
used for tool path [design](https://zero.sci-hub.se/2496/cb390d406cc077ef156deb76b34099af/desantiago-perez2013.pdf#lb0030).  
A notebook on bezier is available in the notebook folder. Bezier is not used as there is no DSP or feedback at the moment.
The controller gets a number of points along curve. The curve is divided in segments and this 
segment is approximated with a polynomal of third degree. There are no multipliers, DSP,
on the ICE40HX4k chip used for this project. As a result, it is has not been implemented.

## Parameters
The following parameters describe the system.  
| parameter | description |
|---|---|
| RPM | revolutions per minute of the rotor |
| Start% | fraction of period where scanline starts |
| End% | fraction of period where scanline stops |
| SPINUP_TIME | seconds system waits for the rotor to stabilize speed |
| STABLE_TIME | seconds system tries to determine position laser with photodiode |
| FACETS | number of polygon facets|
| DIRECTION | exposure direction, i.e. forward or backward |
| SINGLE_LINE | system exposes fixed pattern, i.e. line|
| SINGLE_FACET | only one of the facets is used|
  
Using the above, the code determines the number of bits in a scanline. Via a serial port interface the user can push data to the scanner.
A line is preceded with a command which can be SCAN or STOP. The data is stored on the chip in block ram. 
Once turned on, the scanner reads out this memory via First In First Out (FIFO).
  
# Brief Description
The controller sends over a command with a word to the peripheral which updates the motor state.
The command is 8 bits long and the word 64 bits. Only for write commands, word is not empty.
Typically, the word received by the host is empty unless the memory is full or a read command is issued.

# Commands
The following commands are possible;
| command | reply |
|---|---|
| POSITION | get position of all motors |
| READ | get state of the peripheral and settings of certain pins |
| START | enable execution of instructions stored in SRAM |
| STOP | halt execution of instructions stored in SRAM |
| WRITE | sent over an instruction and store it in SRAM |


## Write
A write commmand is followed by an instruction which is placed in the SRAM.
If the dispatcher is enabled, these instructions are carried out unless an error is raised.
A word can often not store all information for an instruction. So an instruction 
consists out of multiple commands and words in series.
If prior to the sequence, the memory is already full or there is a parsing error, a status word is sent back.
If the reply is zero, the peripheral is operating normally. The information to be sent over is indicated for
each instruction

### Move instruction
| data | number of bytes | description
|---|---|---|
| INSTRUCTION | 1 | type of instructions, here move instruction
| TICKS | 7 | number of ticks in a move, cannot be larger than TICKS_MOVE, i.e. 10_000
| C00 | 8 | motor 0, coeff 0
| C01 | 8 | motor 0, coeff 1
| C02 | 8 | motor 0, coeff 2

The motor will then the follow the path, coef_0 * t + coef_1 * t^2 + coef_2 * t^3.
The coefficients can be interpreted as; velocity, acceleration and jerk. These are slightly different.
If the position is x, then in the formula x = v*t + 1/2*a*t^2 + 1/3*1/2*b*t^3 ; v, a and b are the velocity
acceleration and jerk respectively.
The trajectory of a motor is divided in multiple segments where a segment length is typically 10_000 ticks. 
If is longer, it is repeated. If it is shorter, this is communicated by setting ticks to lower than 10_000.
If multiple motors are used; TICKS, C00, C01, C02 are repeated.
Step speed must be lower than 1/2 oscillator speed (Nyquist criterion).
For a [typical stepper motor](https://blog.prusaprinters.org/calculator_3416/) with 400 steps per mm,
max speed is 3.125 m/s with an oscillator frequency of 1 MHz.
If other properties are desired, alter max_ticks per step, bit_length or motor sampling frequency.
The default motor sampling frequency is 1 MHz.

### Pin instruction
| data | number of bytes | description
|---|---|---|
| INSTRUCTION | 1 | type of instructions, here set pin instruction
| PINS | 7 | Last byte set pins. The last bits set polygon, laser0, laser1

A user can read but not write directly to pins. This ensures that the host
can establish precedence between instructions.

### Laserline instruction
| data | number of bits | description
|---|---|---|
| INSTRUCTION | 8 | type of instructions, here start or stop scanline
| DIRECTION | 1 | scanning direction
| TICKSPERSTEP | 55 | ticks per half period step
| DATA | 64 | information for lasers in chunks of 8 bytes

A user can read but not write directly to pins. This ensures that the host
can establish precedence between instructions.

## Detailed description
Look at the test folders and individually tests on how to use the code. The whole scanhead can be simulated virtually. 
As such, a scanner is not needed.

## Limitations 
System is for writing to, i.e. exposing, a substrate. Reading should also be possible to enable optical coherence tomagraphy.  
System has no link for LIDAR measurements, circuit can be found [here](https://hackaday.io/project/163501-open-source-lidar-unruly).  
The FPGA controls all the stepper motors. At the moment it is not possible to use GCODE or apply acceleration profiles.
Add maximum-length linear-feedback shift register sequence and CRC check.
If you do a sequence of very short moves, e.g. 10 steps, you might notice high-latency due to SPI communcation. 

<!-- 
TODO:
  add docs
 -->
