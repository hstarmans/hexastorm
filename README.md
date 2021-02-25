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
| STOP | halt the execution of gcode |
| GCODE | sent over the GCODE instruction |

## GCODE command
The word of a GCODE command cannot store all the instructions. So a GCODE instruction 
consists out of multiple commands in series.
If prior to the sequence, the memory is already full or there is a parsing error, a status word is sent back.
If the reply is zero, the peripheral is operating normally. The following information must be sent over;
| data | number of bytes | description
|---|---|---|
| COMMAND | 1 | type of commmand, to allow other commands than GCODE
| DIRECTION | 1 | direction of motors
| AUX | 2 | auxilliary bits, to enable lights etc.
| ACCELERATION | 2 | acceleration
| LOOPS_ACCEL | 2 | loops spend in acceleration
| LOOPS_TRAVEL | 2 | loops spend in travel
| DECELERATION | 2 | deceleration
| LOOPS_DECEL | 2 | loops spend in deceleration

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
