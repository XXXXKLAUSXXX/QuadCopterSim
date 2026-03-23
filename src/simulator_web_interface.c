//------------------------------------------------------------------------------
//  Simulate a quadcopter online using WASM
//
//  Author: Andrea Pavan
//  License: MIT
//------------------------------------------------------------------------------
#include <stdio.h>
#include <emscripten/emscripten.h>
#include <time.h>
// #include "simulator.c" // Redundant

// #include "./exampleController.h"
#include "./standardController.h"

double state_new[17];
double state_previous[17] = {
    battery_capacity, // battery charge Cb (coulomb)
    0.0,              // motor #1 speed ω1 (rad/s)
    0.0,              // motor #2 speed ω2 (rad/s)
    0.0,              // motor #3 speed ω3 (rad/s)
    0.0,              // motor #4 speed ω4 (rad/s)
    0.0,              // roll angle ϕ (rad)         //5
    0.0,              // pitch angle θ (rad)
    0.0,              // yaw angle ψ (rad)
    0.0,              // position x (m)
    0.0,              // position y (m)
    0.1,              // altitude z (m)
    0.0,              // roll rate ∂ϕ/∂t (rad/s)
    0.0,              // pitch rate ∂θ/∂t (rad/s)
    0.0,              // yaw rate ∂ψ/∂t (rad/s)
    0.0,              // horizontal speed Ux (m/s) // 14
    0.0,              // horizontal speed Uy (m/s)
    0.0               // vertical speed Uz (m/s)
};
double pilot_input[4] = {0.5, 0.5, 0.5, 0.5};
double motors_pwm[4] = {0.5, 0.5, 0.5, 0.5};
double gyro[3] = {0.0, 0.0, 0.0};
double accel[3] = {0.0, 0.0, 0.0};
double altitude = 0.1;
double simulation_time = 0.0;
double dt = 1e-3;

EMSCRIPTEN_KEEPALIVE
void set_pilot_input(double u1, double u2, double u3, double u4)
{
    // printf("Set new pilot input: %f, %f, %f, %f\n", u1, u2, u3, u4);
    pilot_input[0] = u1;
    pilot_input[1] = u2;
    pilot_input[2] = u3;
    pilot_input[3] = u4;
}

EMSCRIPTEN_KEEPALIVE
double retrieve_state_variable(int idx)
{
    if (idx < 0 || idx >= 17)
    {
        return -1;
    }
    return state_new[idx];
}

EMSCRIPTEN_KEEPALIVE
double retrieve_simulation_time()
{
    return simulation_time;
}

EMSCRIPTEN_KEEPALIVE
void simulate(double tf)
{
    // printf("Simulation started\n");
    // clock_t time0 = clock();
    gyro[0] = state_previous[11];
    gyro[1] = state_previous[12];
    gyro[2] = state_previous[13];

    accel[0] = state_previous[14];
    accel[1] = state_previous[15];
    accel[2] = state_previous[16];

    altitude = state_previous[10];

    controller_p_acro(motors_pwm, pilot_input, gyro, accel, altitude);
    int imax = (int)round(tf / dt);
    for (int i = 1; i < imax; ++i)
    {
        // advance simulation over time
        propagate_state(state_new, state_previous, motors_pwm, dt);

        // fully-discharged battery
        if (state_new[0] <= 0.0)
        {
            state_new[0] = 0.0; // Cb=0
            state_new[1] = 0.0; // shut down motor #1
            state_new[2] = 0.0; // shut down motor #2
            state_new[3] = 0.0; // shut down motor #3
            state_new[4] = 0.0; // shut down motor #4
        }

        // on the ground
        if (state_new[10] <= 0.0)
        {
            state_new[10] = 0.0;    // z=0.0
            state_new[16] = 0.0;    // Uz=0
            state_new[5] = 0.0;     // ϕ=0
            state_new[6] = 0.0;     // θ=0
            state_new[15] *= 0.995; // slow down Uy
            if (fabs(state_new[15]) <= 0.01)
            {
                state_new[15] = 0.0;
            }
            state_new[14] *= 0.995; // slow down Ux
            if (fabs(state_new[14]) <= 0.01)
            {
                state_new[14] = 0.0;
            }
        }

        // update previous state
        for (int j = 0; j < 17; ++j)
        {
            state_previous[j] = state_new[j];
        }

        // update controller at 250Hz (every 4ms)
        if (i % 4 == 0)
        {
            gyro[0] = state_new[11];
            gyro[1] = state_new[12];
            gyro[2] = state_new[13];
            
            accel[0] = state_previous[14];
            accel[1] = state_previous[15];
            accel[2] = state_previous[16];

            altitude = state_previous[10];

            controller_p_acro(motors_pwm, pilot_input, gyro, accel, altitude);
        }
    }
    // clock_t time1 = clock();
    // float elapsed_time = (float) (time1-time0)/CLOCKS_PER_SEC;
    // printf("Elapsed time: %f\n", elapsed_time);
    simulation_time += tf;
    // printf("Simulation time: %f\n", simulation_time);
    // printf("Simulation finished\n");
}

int main()
{
    printf("WASM Module simulator_web_interface running\n");
    return 0;
}
