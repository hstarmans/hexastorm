# FPGA G

The goal of this project is to replace the PRU core of [BeagleG](https://github.com/hzeller/beagleg) with a FPGA core.

# Brief Description
The controller sends over commands to the peripheral which updates the motor state.
The peripheral indicates the controller if the memory is full or busy.
Busy is a limitation of the current implemention (which can be fixed).

# Detailed Description
The controller sends over 16 bytes via SPI.

r2 register holds position but r1 is increased.
it seems to read out all the travel params


IN the pru they are kept in a ring of 16 instructions

An instruction is made up out of the following
QUEUE_ELEMENT_SIZE
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


Notes:
 ideally you hack something in the queye so you can step through program
