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
| START | enable execution of instructions stored in SRAM |
| STOP | halt execution of instructions stored in SRAM |
| WRITE | sent over an instruction and store it in SRAM |

At the moment, only the move instruction is supported.

## Move instruction
A word cannot store all information for a move instruction. So a MOVE instruction 
consists out of multiple commands and words in series.
If prior to the sequence, the memory is already full or there is a parsing error, a status word is sent back.
If the reply is zero, the peripheral is operating normally. The following
information must be sent over;
| data | number of bytes | description
|---|---|---|
| COMMAND | 1 | type of instructions, to allow other instructions then move
| AUX | 2 | auxilliary bits, to enable lights etc.
| TICKS | 4 | number of ticks in a move
| C00 | 4 | motor 0, coeff 0
| C01 | 4 | motor 0, coeff 1
| C02 | 4 | motor 0, coeff 2

The motor will then the follow the path, coef_0 * t + coef_1 * t^2 + coef_2 * t^3.
The coefficients can be interpreted as; velocity, acceleration and jerk.
The trajectory of a motor is divided in multiple segments where a segment length is typically 100_000 ticks, 
i.e 0.1 seconds. If is longer, it is repeated. If it is shorter, this is communicated by setting ticks to lower than 100_000.
If multiple motors are used; ticks, C00, C01, C02 are repeated.

# Installation
Although deprecated tools are installed via apio;
```
export PATH=/home/pi/.local/bin:$PATH
export PATH=/home/pi/.apio/packages/toolchain-yosys/bin:$PATH
export PATH=/home/pi/.apio/packages/toolchain-ice40/bin:$PATH
``` 

## Background
Splines, Bezier, B-splines, and NURBS (Non-Uniform Rational B-splines) curves are the common parametric techniques 
used for tool path [design](https://zero.sci-hub.se/2496/cb390d406cc077ef156deb76b34099af/desantiago-perez2013.pdf#lb0030).  
A notebook on bezier is available in the notebook folder.
This is finally all ignored. The controller gets a number of points along curve. The curve is divided in segments and this 
segment is approximated with a polynomal of third degree.
