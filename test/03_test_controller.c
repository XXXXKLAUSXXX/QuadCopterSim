/*******************************************************************************
    Testing program for the proportional controller

    How to run:
    gcc 03_test_controller.c -o 03_test_controller -lm -Wall -Wextra
    ./03_test_controller

    Author: Andrea Pavan
    License: MIT
*******************************************************************************/
#include <math.h>
#include <stdio.h>
#include "../src/simulator.c"

int main() {
    //simulation parameters
    double dt = 0.001;              //simulation timestep (s)
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
    double motors_pwm[4] = {0.5, 0.5, 0.5, 0.5};
    double pilot_input[4] = {0.5, 0.5, 0.7, 0.5};
    double gyro[3] = {0.0, 0.0, 0.0};
    controller_p_acro(motors_pwm, pilot_input, gyro);
    for (int i=1; i<10000; ++i) {
        //time integration
        //assumption: the pilot input is constant over a timestep
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

        //update controller at 250Hz (every 4ms)
        pilot_input[0] = 0.5;
        pilot_input[1] = 0.5;
        pilot_input[2] = 0.5;
        pilot_input[3] = 0.5;
        if (i%4 == 0) {
            //set thrust pilot input
            if (i < 1500) {
                pilot_input[2] = 0.7;
            }
            else if (i < 3000) {
                pilot_input[2] = 0.2;
            }
            else if (i < 3500) {
                pilot_input[2] = 0.3;
            }
            else if (i < 4000) {
                pilot_input[2] = 0.4;
            }
            else if (i < 5000) {
                pilot_input[2] = 0.57;
            }
            else if (i < 6000) {
                pilot_input[2] = 0.75;
            }
            else if (i < 7000) {
                pilot_input[2] = 0.8;
            }
            else if (i < 8000) {
                pilot_input[2] = 0.51;
            }
            else {
                pilot_input[2] = 0.4;
            }

            //set yaw pilot input
            if (i >= 3000 && i < 5000) {
                pilot_input[3] = 0.7;
            }
            else if (i >= 5000 && i < 7000) {
                pilot_input[3] = 0.3;
            }
            else {
                pilot_input[3] = 0.5;
            }

            //set roll pilot input
            if (i >= 3000 && i < 6000) {
                pilot_input[0] = 0.7;
            }
            else if (i >= 6000 && i < 9000) {
                pilot_input[0] = 0.3;
            }
            else {
                pilot_input[0] = 0.5;
            }
            
            //run control
            gyro[0] = state_new[11];
            gyro[1] = state_new[12];
            gyro[2] = state_new[13];
            controller_p_acro(motors_pwm, pilot_input, gyro);
        }

        //print results every 0.1 seconds
        double t = i*dt;
        if (i%100 == 0) {
            printf("z(%f) = %f m\n", t, state_new[10]);
        }
    }
    return 0;
}
