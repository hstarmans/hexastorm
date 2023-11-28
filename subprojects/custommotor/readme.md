# Tuning a motor

## Rotation mode
Fix the state machine in rotation and record the direction of rotation.
If the motor does not rotate, reduce the distance between the magnets and the PCB
as the force is dependent on the square of the distance.
Voltage should not exceed 9 volts as the driver will blow up.

## Checking hall sensors
Place the script test_electrical.py in hallfilter mode. 
Record how a rotation takes place; e.g. red (1), cyan (5), darblue (4), lightblue (6), green (2), yellow (3) and red.
If you miss states, e.g. don't have yellow but black instead of yellow. Your motor might not be well aligned.
In hall filter mode, we map this to the motor state; 1-->1, 5-->2, 4-->3, 6-->4, 2-->5, 3-->6.
The motor transitions from rotation mode to hall sensor feedback mode. If the motor does not rotate, you might need to shift the whole thing, e.g. 1-->2, 5-->3 etc. or simply change the place where pulses are placed.

##  Imphall mode
The distances between the states is not 30 degrees. As a result, the pulsing of the motor is not optimal.
This is corrected by the imphall mode. To collect the distances run the hall filter mode. Ensure the measurement is
repeatable and enter the values in the driver.
Collect the angles and ensure the spacing is uniform by tuning the beta coefficient.
You should have 0.167 measurements per angular bin.

## Measurements noise
Measurement noise is filtered out by reducing the number of samples and ignoring state 0 and state 7.
Sometimes an additional state is filtered if the trigger is not proper.

## PID 
First, let the thing run wihout a PID controller. Ensure you can reach a speed higher than the target value.
The speed can then be lowered by tuning the PID controller and using a guide like [here](https://tlk-energy.de/blog-en/practical-pid-tuning-guide).
I am able to get an accuracy around one percent. 
Noise is most likely a result of the inaccurate readings with the Hall sensor.
