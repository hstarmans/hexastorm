# FPGA G

The goal of this project is to replace the PRU core of [BeagleG](https://github.com/hzeller/beagleg) with a FPGA core.
This is to combine prism scanner with other techniques.

# Brief Description
The controller sends over a command with a word to the peripheral which updates the motor state.
The command is 8 bits long and the word 32 bits.

# Commands
The following commands are possible;
| command | reply |
|---|---|
| STATUS | send back the status of the peripheral|
| START | enable execution of gcode |
| STOP | halt execution of gcode |
| GCODE | sent over the GCODE instruction |

## GCODE command
The word of a GCODE command cannot store all the instructions. So a GCODE instruction 
consists out of multiple commands in series.
If prior to the sequence, the memory is already full or there is a parsing error, a status word is sent back.
If the reply is zero, the peripheral is operating normally. In the case of 1 motor, and bezier of the order 3, the following
commands must be sent over;
| data | number of bytes | description
|---|---|---|
| COMMAND | 1 | type of commmand, to allow other commands than GCODE
| AUX | 2 | auxilliary bits, to enable lights etc.
| B00 | 4 | bezier coeff 0, motor 0
| B01 | 4 | bezier coeff 1, motor 0
| B02 | 4 | bezier coeff 2, motor 0
| B03 | 4 | bezier coeff 3, motor 0
| B04 | 4 | bezier coeff 4, motor 0

# Accucuracy
If scale is set to 1 micron, position is defined in 32 bit signed, the range is +/- 2147 meters.
In de casteljau's, time is a float between 0 an 1. Let's assume, 10 samples are taken per second,
the update frequency is 1 MHZ. The float needs to be able to carry 100E3. Looking at the formula
the max is (-2*pow(100E3, 2)) which is accounted for in 32 bit arithmetic.
To account for the floats; we do a bitshit of 17 as (2,17) equals 131K.
It might be an option to set B00 equal to zero, set the scale to steps... but this is for later..

## Algorithm
Splines, Bezier, B-splines, and NURBS (Non-Uniform Rational B-splines) curves are the common parametric techniques 
used for tool path [design](https://zero.sci-hub.se/2496/cb390d406cc077ef156deb76b34099af/desantiago-perez2013.pdf#lb0030).  
Only bezier curves are supported at the moment. The way Bezier is implemented is outlined in the notebook bezier.

# Installation
 Although deprecated tools are installed via apio;
```
export PATH=/home/pi/.local/bin:$PATH
export PATH=/home/pi/.apio/packages/toolchain-yosys/bin:$PATH
export PATH=/home/pi/.apio/packages/toolchain-ice40/bin:$PATH
``` 
