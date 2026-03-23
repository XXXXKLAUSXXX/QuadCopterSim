# Setup env
Install EEMC compiler
# Needs
We need:
    Gyroscope input
    Accelerometer input
    Motor output

# Up next
programming in a PID controller
 Reading HTML values for PID settings
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

# Cookbook
## Build and run
## Get variables from the C programs to the HTML page
Done via index.js. line 180 //Update Primary Flight Display. Make sure to use cntrol+r to clear the cache.
