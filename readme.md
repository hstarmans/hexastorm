# Laserscanner
Implementation of a laserscanner on a FPGA. In a high-speed polygon scanner system, the laser is deflected by a rotating prism or reflective mirror. 
The position of the laser is determined via a sensor such as a photodiode.
<img src="https://cdn.hackaday.io/images/7106161566426847098.jpg" align="center" height="300"/> 
<br>
Code is tested on the system shown in the image above, branded as [Hexastorm](https://www.hexastorm.com). 
The bill of materials (BOM) and links to FreeCad and PCB designs can be found on [Hackaday](https://hackaday.io/project/21933-open-hardware-fast-high-resolution-laser).

## Install Notes
Install migen
```python
pip3 install -e 'git+http://github.com/m-labs/migen.git#egg=migen'
```
Install [litex](https://github.com/enjoy-digital/litex), make a special folder for this installation
For setting the power to laser over ic, you need to install smbus. <br>
Install the FPGA toolchain for [ICE40](http://www.clifford.at/icestorm/).
An easier way is apio, but nextpnr is deprecated and doesn't support all options.
```console
pip install -U apio
apio install yosys
apio install ice40
```
Example command
```console
apio raw "yosys -q"
```
FPGA currently used is ICE40 with [Icestorm](http://www.clifford.at/icestorm/) flow.
Support for other FPGAs is possibly provided by [symbiflow](https://symbiflow.github.io/)

## Parameters
The user has to define the following parameters;

| parameter | description |
|---|---|
| RPM | revolutions per minute of the rotor |
| Start% | fraction of period where scanline starts |
| End% | fraction of period where scanline stops |
| SPINUP_TIME | seconds system waits for the rotor to stabilize speed |
| STABLE_TIME | seconds system tries to determine position laser with photodiode |
| FACETS | number of polygon facets|
| DIRECTION | exposure direction, i.e. forward or backward |
| SYNCSTART | 1-SYNCSTART fraction where laser is turned on to trigger photodiode |
| JITTER_THRESH | allowed fractional jitter per period |
| SINGLE_LINE | system exposes fixed pattern, i.e. line|
<br>
Using the above, the code determines the number of bits in a scanline. Via a serial port interface the user can push data to the scanner.
This data is stored on the chip in block ram. If turned on the scanner reads out this memory via First In First Out (FIFO).
The laser is turned on, only if the bit is equal to one. If the user tries to read when the memory is empty or tries to write when the memory is full a memory full/empty error is shown.

## Commands
| command | reply |
|---|---|
| STATUS | retrieve the error state and state of the laser scanner. This is the default reply of each command.|
| START | enable the scanhead |
| STOP | stop the scanhead |
| MOTORTEST | enable the motor |
| LASERTEST | turn on the laser|
| LINETEST | turn on the laser and the motor|
| PHOTODIODETEST | turn on motor, laser and turn off if photodiode is triggered|
| WRITE_L | next 8 bytes will be stored in memory |
| READ_D | retrieve debug information, not used |
<br>
A typical use case is as follows. The user retrieves the status of the scanhead. This should be stop.
A motortest is executed. If after a second the status is determined and it does not equal stop; there is a hardware malfunction.
The user uploads data to the memory until the memory is full and the error memory full is returned.
The user turns on the scanhead and the laser starts writing the information to the substrate.  
While the head is on, the user keeps pushing data to the memory.

## Detailed description
The whole scanhead can be simulated virtually. As such, a detailed description is available via the virtual test of the system

## Limitations
Only one of the 32 block rams is used. <br>
The laser can be pulsed at 100 MHZ but data can only be streamed to the laser scanner at 25 megabits per second. <br>
Parameters can not be changed on the fly. The binary has to be recompiled and uploaded to the scanhead. This is typically fast, i.e. seconds. <br>
The current implentation is targeted at a system with one laser bundle and for writing to, i.e. exposing, a substrate. Reading should however also be possible.<br>
The FPGA controls all the stepper motors. At the moment it is not possible to use GCODE or apply acceleration profiles. <br>
<br>
Most of these implementation can be removed by improving the code. The current focus is on a proof of principle and creating a FPGA gcode parser using [beagleg](https://github.com/hzeller/beagleg).


## Other notes
### Migen examles
Examples used to gain experience with migen.

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
<!-- 
TODO:
  try to compile binary
  write hardware test case
  add photodiode synt to the end of test with write and also make sure you can write to multiple lines
  add virtual test for single line
  single line now keeps stationary at a fixed 8 bit pattern
  add virtual test for single facet
  add movement, the head should determine wether it has to move after a line. You need to add this encoding.
  replace migen with nmigen
 -->
