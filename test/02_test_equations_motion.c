/*******************************************************************************
    Testing program for the equations of motion

    How to run:
    gcc 02_test_equations_motion.c -o 02_test_equations_motion -lm -Wall -Wextra
    ./02_test_equations_motion

    Author: Andrea Pavan
    License: MIT
*******************************************************************************/
#include <math.h>
#include <stdio.h>
#include "../src/simulator.c"

int main() {
    //simulation parameters
    double dt = 0.01;               //simulation timestep (s)
    double tf = 10.0;               //simulation end time (s)

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
    printf("z(%f) = %f m\n", 0.0, state_previous[10]);
    double state_new[17];
    double motors_pwm[4] = {1.0, 1.0, 1.0, 1.0};
    for (double t=dt; t<=tf; t+=dt) {
        //time integration
        propagate_state(state_new, state_previous, motors_pwm, dt);

        //fully-discharged battery
        if (state_new[0] <= 0) {
            state_new[0] = 0;       //Cb=0
        }

        //on the ground
        if (state_new[10] <= 0) {
            state_new[10] = 0;      //z=0
            state_new[16] = 0;      //Uz=0
        }

        //update previous state
        for (int j=0; j<17; ++j) {
            state_previous[j] = state_new[j];
        }

        //print results every 0.1 seconds
        if (fabs(t/0.1 - round(t/0.1)) < 1e-6) {
            printf("z(%f) = %f m\n", t, state_new[10]);
        }
    }
    return 0;
}
