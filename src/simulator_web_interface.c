//------------------------------------------------------------------------------
//  Simulate a quadcopter online using WASM
//
//  Author: Andrea Pavan
//  License: MIT
//------------------------------------------------------------------------------
#include <stdio.h>
#include <emscripten/emscripten.h>
#include "simulator.c"


double state_new[17];
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
    2.0,                        //altitude z (m)
    0.0,                        //roll rate ∂ϕ/∂t (rad/s)
    0.0,                        //pitch rate ∂θ/∂t (rad/s)
    0.0,                        //yaw rate ∂ψ/∂t (rad/s)
    0.0,                        //horizontal speed Ux (m/s)
    0.0,                        //horizontal speed Uy (m/s)
    0.0                         //vertical speed Uz (m/s)
};
double pilot_input[4] = {0.5, 0.5, 0.5, 0.5};
double motors_pwm[4] = {0.5, 0.5, 0.5, 0.5};
double gyro[3] = {0.0, 0.0, 0.0};


EMSCRIPTEN_KEEPALIVE
void set_pilot_input(double u1, double u2, double u3, double u4) {
    //printf("Set new pilot input: %f, %f, %f, %f\n", u1, u2, u3, u4);
    pilot_input[0] = u1;
    pilot_input[1] = u2;
    pilot_input[2] = u3;
    pilot_input[3] = u4;
}


EMSCRIPTEN_KEEPALIVE
double retrieve_state_variable(int idx) {
    if (idx < 0 || idx >= 17) {
        return -1;
    }
    return state_new[idx];
}


EMSCRIPTEN_KEEPALIVE
void simulate() {
    //printf("Simulation started\n");
    double dt_simulation = 1e-3;
    gyro[0] = state_previous[11];
    gyro[1] = state_previous[12];
    gyro[2] = state_previous[13];
    controller_p_acro(motors_pwm, pilot_input, gyro);
    //printf("Motors pwm: %f, %f, %f, %f\n", motors_pwm[0], motors_pwm[1], motors_pwm[2], motors_pwm[3]);
    for (int i=1; i<25; ++i) {
        //advance simulation over time
        propagate_state(state_new, state_previous, motors_pwm, dt_simulation);
        
        //fully-discharged battery
        if (state_new[0] <= 0) {
            state_new[0] = 0;           //Cb=0
        }
        
        //on the ground
        if (state_new[10] <= 0.3) {
            state_new[10] = 0.3;        //z=0.3
            state_new[16] = 0;          //Uz=0
        }
        
        //update previous state
        for (int j=0; j<17; ++j) {
            state_previous[j] = state_new[j];
        }
        
        //update controller at 250Hz (every 4ms)
        if (i%4 == 0) {
            gyro[0] = state_new[11];
            gyro[1] = state_new[12];
            gyro[2] = state_new[13];
            controller_p_acro(motors_pwm, pilot_input, gyro);
        }
    }
    //printf("Simulation finished\n");
}

int main() {
    printf("WASM Module simulator_web_interface running\n");
    return 0;
}

