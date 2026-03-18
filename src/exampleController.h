#ifndef controllerTarg
#define controllerTarg "example"

#include "./simulator.c"

double oldAltitude = 0;

// proportional controller for acro/roll mode.
// CONTROLLER_P_ACRO calculates the desired rates from the sticks and applies a
// proportional controller to calculate the motors' duty cycles
// Duty cycle for each motor?
// sticks x,y,x,y?
// Gyro XYZ rot?
void controller_p_acro(double duty_cycle[4], double sticks[4], double gyro[3], double accel[3], double altitude)
{
    double altitudeChange = altitude - oldAltitude;

    for (int i = 0; i < 4; i++)
    {
        if (altitude < 3)
        {
            duty_cycle[i] = 0.7;
        }
        else if (altitude > 9)
        {
            duty_cycle[i] = 0.5;
        }
        else
        {
            duty_cycle[i] = 0.6;
        }
    }

    // Update variables to store info
    oldAltitude = altitude;
}

#endif