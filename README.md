# FPGA G

The goal of this project is to replace the PRU core of [BeagleG](https://github.com/hzeller/beagleg) with a FPGA core.

# Brief Description
The controller sends over commands to the peripheral which updates the motor state.
The peripheral indicates the controller if the memory is full or busy.
Busy is a limitation of the current implemention (which can be fixed).

# Detailed Description
On the pru 16 instructions can be kept in a ring.
Each instructions is 30 bytes (QUEUE_ELEMENT_SIZE) long.
The current idea is to decouple the retrieval and processing of instructions.
The FPGA retrieves the instructions and stores them locally on the chip. Later these instructions are parsed into
signals for the motor.

An instruction is made up out of 2 bytes.

byte0 : state can be empty, filled, exit and abort.
byte1 : direction bit
travel parameters
    2 bytes  : loops_accel
    2 bytes  : loops_travel
    2 bytes  : loops_decel
    2 bytes  : aux
    4 bytes  : accel_series_index
    4 bytes  : hires_accel_cycles
    4 bytes  : travel_delay_cycles
    4 bytes  : fractions
    2 bytes  : jerk_start
    2 bytes  : jerk_stop


register 28 is status
r28.b3 is queue position (the bottom 3 bytes are zero)


Steps
 - process instruction
 if not empty
 - set direction bits (SetDirections)  --> simple
 - copy travel parameters  --> medium complexity there are a lot
 - set auxiliary bits (SetAuxBits) --> Simple
 - initialize motor state with 0
 - update status register (r28)

  Step gen is a loop
  before step gen
 - checkforestop ---> strange can't find definition
   if case it is aborted
  otherwise the staes are updated with fractions and gpio is set
  Delay is calculated (complex), remainder is party put in "state register"
  You wait for the delay and execute again the step gen if zero you are done and start waiting for process once more
  

# Steps:
- Make implementation
- Write test suite implementation
- Compile beagleg
- Place in your on module for the PRU part and debug this with present google test


# Checks
Beagle G prefers processing of 16 bytes per transfer. You have been using 8 bytes.
You need to work with old modules.



 # Installation
 Although deprecated tools are installed via apio;
```
export PATH=/home/pi/.local/bin:$PATH
export PATH=/home/pi/.apio/packages/toolchain-yosys/bin:$PATH
export PATH=/home/pi/.apio/packages/toolchain-ice40/bin:$PATH
``` 