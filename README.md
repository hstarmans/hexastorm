# FPGA G

The goal of this project is to replace the PRU core of [BeagleG](https://github.com/hzeller/beagleg) with a FPGA core.

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
The word of a GCODE command cannot store all the instructions. So a GCODE project instruction 
consits out of multiple commands in series.
If prior to the sequence, the memory is already full or there is a parsing error, a status word is sent back.
If the reply is zero, the peripheral is operating normally. The following information must be sent over;
| data | number of bytes | description
|---|---|
| COMMAND | 1 | type of commmand, to allow other commands than GCODE
| DIRECTION | 1 | direction of motors
| AUX | 2 | auxilliary bits, to enable lights etc.
| ACCELERATION | 2 | acceleration
| LOOPS_ACCEL | 2 | loops spend in acceleration
| LOOPS_TRAVEL | 2 | loops spend in travel
| DECELERATION | 2 | deceleration
| LOOPS_DECEL | 2 | loops spend in deceleration

## Speed calculation
It is assumed that the motor accelerates with a uniform acceleration for a given number of steps.
The circuit operates at a static frequency say, 100 MHZ. During the momevent, the circuit must calculate the current speed.
An alternative is to send over the speed, but it is assumed that this is too slow and would require too much computation at the host.
This acceleration speed is calculated with the following formula, in correspondence with [article](https://www.embedded.com/generate-stepper-motor-speed-profiles-in-real-time/).
![c_i = c_{i-1}-\frac{2C_{i-1}}{4n_i+1}](http://www.sciweavers.org/tex2img.php?eq=c_i%20%3D%20c_%7Bi-1%7D-%5Cfrac%7B2C_%7Bi-1%7D%7D%7B4n_i%2B1%7D%20&bc=White&fc=Black&im=jpg&fs=12&ff=arev&edit=0)  
The division is implemented on the FPGA via the Euclidean algorithm. The number of steps is equalt to the bit width.
So for a bit width of 32 the maximum update frequency for a frequency of 100 MHz is 3.1 MHz.
# Installation
 Although deprecated tools are installed via apio;
```
export PATH=/home/pi/.local/bin:$PATH
export PATH=/home/pi/.apio/packages/toolchain-yosys/bin:$PATH
export PATH=/home/pi/.apio/packages/toolchain-ice40/bin:$PATH
``` 