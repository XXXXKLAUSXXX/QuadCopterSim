#ifndef controllerTarg 
#define controllerTarg "example"

#include "./simulator.c"

// proportional controller for acro/roll mode.
// CONTROLLER_P_ACRO calculates the desired rates from the sticks and applies a
// proportional controller to calculate the motors' duty cycles
// Duty cycle for each motor?
// sticks x,y,x,y?
// Gyro XYZ rot?
void controller_p_acro(double duty_cycle[4], double sticks[4], double gyro[3], double accel[3], double altitude)
{
    for (int i = 0; i < 4; i++)
    {
        duty_cycle[i] = 1;
    }
}

#endif