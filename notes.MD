# Setup env
Install EEMC compiler
# Needs
We need:
    Gyroscope input
    Accelerometer input
    Motor output

# Up next

Rewriting controller_p_acro to allow a system control.
then might add chaos and noise to inputs and outputs.

# Reading from the sensors
## Altitude
[10]
## Angle
Gyroscope readouts are passed to controller_p_acro
Get these to print? (create elements in Index.html. Index.js to update the variable)
## Acceleration
state_previous[14]-[16] in simulator_web_interface is used to store the speed of the drone.
# Motor output
Motors are driven in simulator.c controller_p_acro using a float ranging from 0 to 1
Write to duty cycle[] to control the motors.