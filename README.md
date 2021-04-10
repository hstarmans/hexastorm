# FPGA G

The goal of this project is to create nmigen core for CNC control. This is needed for my [laser scanner project](https://github.com/hstarmans/hexastorm).  
A proof of concept video is shown below;

[![Proof of concept not showing ](http://img.youtube.com/vi/-0uB2MydtrE/0.jpg)](https://youtu.be/-0uB2MydtrE "Proof of woking code")


# Brief Description
The controller sends over a command with a word to the peripheral which updates the motor state.
The command is 8 bits long and the word 32 bits. Only for write commands, word is not empty.
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
| INSTRUCTION | 1 | type of instructions, here pin instruction
| AUX | 7 | pins to set equal to 1 or 0

A user can read but not write directly to pins. This ensures that the host
can establish precedence between instructions.

# Installation

## FPGA Toolchain
Although deprecated tools are installed via apio;
```
export PATH=/home/pi/.local/bin:$PATH
export PATH=/home/pi/.apio/packages/toolchain-yosys/bin:$PATH
export PATH=/home/pi/.apio/packages/toolchain-ice40/bin:$PATH
``` 
Code partially depends on Luna. Firstarter board can be selected via
```
export LUNA_PLATFORM="FPGAG.board:Firestarter"
```
Signal traces for GTKWave can be generated via;
```
export GENERATE_VCDS=1
```

## Stepper drivers
Install the python wrapper for TMC stepper [drivers](https://github.com/hstarmans/TMCStepper).

# Limitations
Add maximum-length linear-feedback shift register sequence and CRC check.
If you do a sequence of very short moves, e.g. 10 steps, you might notice high-latency due to SPI communcation. 

## Background
Splines, Bezier, B-splines, and NURBS (Non-Uniform Rational B-splines) curves are the common parametric techniques 
used for tool path [design](https://zero.sci-hub.se/2496/cb390d406cc077ef156deb76b34099af/desantiago-perez2013.pdf#lb0030).  
A notebook on bezier is available in the notebook folder. This is finally all ignored. 
The controller gets a number of points along curve. The curve is divided in segments and this 
segment is approximated with a polynomal of third degree. There are no multipliers, DSP,
on the ICE40HX4k chip used for this project. As a result, it is has not been implemented.
