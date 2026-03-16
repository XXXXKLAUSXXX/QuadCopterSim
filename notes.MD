# Needs
We need:
    Gyroscope input
    Accelerometer input
    Motor output

# Reading from the sensors
## Angle
Gyroscope readouts are passed to controller_p_acro
Get these to print maybe?
## Acceleration
state_previous[14]-[16] in simulator_web_interface is used to store the speed of the drone.
# Motor output
Motors are driven in simulator.c controller_p_acro using a float ranging from 0 to 1
Write to duty cycle[] to control the motors.