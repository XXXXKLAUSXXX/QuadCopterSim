/*******************************************************************************
    Testing program for the battery charge

    How to run:
    gcc 01_test_battery_discharge.c -o 01_test_battery_discharge -lm -Wall -Wextra
    ./01_test_battery_discharge

    Author: Andrea Pavan
    License: MIT
*******************************************************************************/
#include <math.h>
#include <stdio.h>
#include "../src/simulator.c"

int main() {
    //simulation parameters
    double dt = 0.01;               //simulation timestep (s)
    double tf = 10.0 * 60;          //simulation end time (s)

    //time propagation
    double state_previous[17] = {
        battery_capacity,           //battery charge Cb (coulomb)
        0.0,                        //motor #1 speed ω1 (rad/s)
        0.0,                        //motor #2 speed ω2 (rad/s)
        0.0,                        //motor #3 speed ω3 (rad/s)
        0.0,                        //motor #4 speed ω4 (rad/s)
        0.0,                        //roll angle ϕ (rad)
        0.0,                        //pitch angle θ (rad)
        0.0,                        //yaw angle ψ (rad)
        0.0,                        //position x (m)
        0.0,                        //position y (m)
        0.0,                        //altitude z (m)
        0.0,                        //roll rate ∂ϕ/∂t (rad/s)
        0.0,                        //pitch rate ∂θ/∂t (rad/s)
        0.0,                        //yaw rate ∂ψ/∂t (rad/s)
        0.0,                        //horizontal speed Ux (m/s)
        0.0,                        //horizontal speed Uy (m/s)
        0.0                         //vertical speed Uz (m/s)
    };
    printf("Cb(%f) = %f mAh\n", 0.0, state_previous[0]/3.6);
    double state_new[17];
    double motors_pwm[4] = {1.0, 1.0, 1.0, 1.0};
    for (double t=dt; t<=tf; t+=dt) {
        //time integration
        propagate_state(state_new, state_previous, motors_pwm, dt);

        //update previous state
        for (int j=0; j<17; ++j) {
            if (j >= 6) {
                state_new[j] = 0;
            }
            state_previous[j] = state_new[j];
        }

        //print results every 60 seconds
        if (fabs(t/60 - round(t/60)) < 1e-6) {
            printf("Cb(%f) = %f mAh\n", t, state_new[0]/3.6);
        }
    }
    return 0;
}
