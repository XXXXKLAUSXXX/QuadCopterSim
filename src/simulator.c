#pragma once
//------------------------------------------------------------------------------
//  This library simulates the nonlinear dynamics of a F450 quadcopter.
//  A simple proportional controller is also implemented, based on a
//  linearized model. The quadcopter is piloted in ACRO mode, prescribing
//  the desired angular rates and without any thrust compensation.
//
//  How to compile:
//  gcc simulator.c -o simulator.so -shared -lm -fPIC -O2 -Wall -Wextra
//
//  Author: Andrea Pavan
//  License: MIT
//------------------------------------------------------------------------------
#include <math.h>
#include <stdio.h>

//constants
const double Jprop = 10.4e-5;               //propeller moment of inertia (kg*m^2)
const double battery_capacity = 5000 * 3.6; //battery capacity (coulomb) - equivalent to 5000mAh
const double Rdist = 0.0506972;             //power distribution resistance (omega)
const double larm = 0.450/2;                //motor distance from CG (m)
const double Ixx = 8.825e-3;                //moment of inertia around x-axis (kg*m^2)
const double Iyy = 8.825e-3;                //moment of inertia around y-axis (kg*m^2)
const double Izz = 14.39e-3;                //moment of inertia around z-axis (kg*m^2)
const double m = 1.08 + 0.395;              //total mass (kg)
const double g = 9.81;                      //gravity acceleration (m/s^2)
const double rho = 1.225;                   //air density (kg/m^3)
const double mu = 1.81e-5;                  //air dynamic viscosity (Pa*s)
const double Aref = 0.1;                    //aerodynamic reference area (m2)
const double Kt = 8.634692591183668e-6;     //thrust-speed coefficient (N/(rad/s)^2)
const double Ktau = 1.106027026514568e-7;   //torque-speed coefficient (N-m/(rad/s)^2)
const double Komega = 919.2300104403735;    //speed-throttle coefficient (rad/s)

//motor properties
const double Kv = 1 / (920 * 3.14159/30);   //speed constant (V/(rad/s))
const double I0 = 0.5;                      //no-load current (A)
const double Rm = 0.142;                    //armature resistance (omega)

//controller properties
const double MAX_RP_RATE = 5.0;             //maximum roll/pitch rate (rad/s)
const double MAX_Y_RATE = 5.0;              //maximum yaw rate (rad/s)
const double Kpphi = 3.0;                   //proportional gain in the pitch axis
const double Kptheta = 3.0;                 //proportional gain in the roll axis
const double Kppsi = 6.0;                   //proportional gain in the yaw axis
double Minv[4][4] = {{0.0, 0.5, -0.25, 0.25},       //inverse mixing matrix
                    {-0.5, 0.0, 0.25, 0.25},
                    {0.0, -0.5, -0.25, 0.25},
                    {0.5, 0.0, 0.25, 0.25}};

//simulation parameters
const double Δt = 0.01;                     //simulation timestep (s)
const double tf = 10.0;                     //simulation end time (s)


//linear interpolation function
//INTERP1 performs linear interpolation to find the value of y at a given x (xq) 
//based on a set of x and y values.
//If xq is outside the range of x values, it returns the y value corresponding
//to the nearest x value.
double interp1(double* x, double* y, int N, double xq) {
    if (xq < x[0]) {
        //query point is lower than the minimum value
        return y[0];
    }
    else if (xq > x[N-1]) {
        //query point is higher than the maximum value
        return y[N-1];
    }
    else {
        //query point is within the interpolating interval
        for (int j=0; j<N-1; ++j) {
            if (x[j] <= xq && xq <= x[j+1]) {
                return y[j] + (xq - x[j]) * (y[j+1] - y[j]) / (x[j+1] - x[j]);
            }
        }
    }
    //this should not happen
    return -1;
}


//calculate motor current
//MOTOR_CURRENT calculates the motor current based on the given speed,
//duty cycle and source voltage.
double motor_current(double speed, double duty_cycle, double source_voltage) {
    double Vavg = source_voltage * duty_cycle;      //average voltage (V)
    double Vemf = (speed + 1e-6) * Kv;              //back-emf voltage (V)
    double I = (Vavg - Vemf) / (Rm + Rdist);        //current (A)
    return I;
}


//calculate motor torque
//MOTOR_TORQUE calculates the motor torque based on the given speed,
//duty cycle and source voltage.
double motor_torque(double speed, double duty_cycle, double source_voltage) {
    double Vavg = source_voltage * duty_cycle;      //average voltage (V)
    double Vemf = (speed + 1e-6) * Kv;              //back-emf voltage (V)
    double I = (Vavg - Vemf) / (Rm + Rdist);        //current (A)
    double Vm1 = Vavg - Rdist * I;
    double Pheat = Rm * I * I + Vemf * I0;          //heat losses (W)
    double Pin = I * Vm1;                           //electrical power (W)
    double Pout = Pin - Pheat;                      //mechanical power (W)
    double τ = Pout / (speed + 1e-6);               //motor torque (N-m)
    return τ;
}


//calculate propeller torque in hovering mode
//PROPELLER_TORQUE_HOVERING calculates the propeller torque at a given speed
double propeller_torque_hovering(double speed) {
    double x[] = {0.0, 104.71975511965977, 209.43951023931953, 314.1592653589793, 418.87902047863906, 523.5987755982989, 628.3185307179587, 733.0382858376183, 837.7580409572781, 942.477796076938, 1047.1975511965977};
    double y[] = {0.0, 0.0019671550966158264, 0.006445775195221762, 0.013165296892561582, 0.022061262436683073, 0.03311123098061027, 0.046312496890310625, 0.06168982012764514, 0.07927706659086173, 0.0991216985576324, 0.12128943565124396};
    int N = sizeof(x) / sizeof(x[0]);
    return interp1(x, y, N, speed);
}

//calculate propeller thrust in hovering mode
//PROPELLER_THRUST calculates the propeller thrust based at the given speed
double propeller_thrust(double speed) {
    double x[] = {0.0, 104.71975511965977, 209.43951023931953, 314.1592653589793, 418.87902047863906, 523.5987755982989, 628.3185307179587, 733.0382858376183, 837.7580409572781, 942.477796076938, 1047.1975511965977};
    double y[] = {0.0, 0.092, 0.369, 0.832, 1.484, 2.325, 3.358, 4.585, 6.010, 7.636, 9.469};
    int N = sizeof(x) / sizeof(x[0]);
    return interp1(x, y, N, speed);
}


//calculate friction torque.
//FRICTION_TORQUE calculates the friction torque of a rotor at a given speed
double friction_torque(double speed) {
    return 0.0140347 * (speed / 104.71975511965977 <= 1 ? speed / 104.71975511965977 : 1);
}


//calculate battery voltage
//BATTERY_VOLTAGE calculates the battery voltage associated with a given state of charge (SoC)
double battery_voltage(double SoC) {
    double x[] = {0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8, 0.85, 0.9, 0.95, 1};
    double y[] = {9.82, 10.83, 11.06, 11.12, 11.18, 11.24, 11.3, 11.36, 11.39, 11.45, 11.51, 11.56, 11.62, 11.74, 11.86, 11.95, 12.07, 12.25, 12.33, 12.45, 12.6};
    int n = sizeof(x) / sizeof(x[0]);
    return interp1(x, y, n, SoC);
}


//estimate aerodynamic drag force
//AERODYNAMIC_DRAG_FORCE calculates the aerodynamic drag at a given airspeed
void aerodynamic_drag_force(double drag_force[3], double speed_vector[3]) {
    double Uinf = sqrt(speed_vector[0]*speed_vector[0] + speed_vector[1]*speed_vector[1] + speed_vector[2]*speed_vector[2]);  //airspeed magnitude (m/s)
    double Re = rho * sqrt(Aref/3.14159) * Uinf / mu;      //Reynolds number
    Re += 1.0;              //avoid division by zero in the subsequent R22 formula
    double CD = 0.40;       //drag coefficient of a sphere (turbulent separation)
    if (Re < 350000.0) {
        //use R22 formula without correction
        //to estimate the drag coefficient with laminar separation
        CD = (1.0 + (Re*3/16.0) / (1.0 + (Re*19/240.0) / (1.0 + Re*1/122.0))) * 24/Re;
    }
    drag_force[0] = 0.5 * rho * speed_vector[0] * Uinf * Aref * CD;
    drag_force[1] = 0.5 * rho * speed_vector[1] * Uinf * Aref * CD;
    drag_force[2] = 0.5 * rho * speed_vector[2] * Uinf * Aref * CD;
}


//calculate the right-hand side of the ODE system of equations
//RHS calculates the derivatives of the state variables based on the given current state and motors' duty cycles
void rhs(double derivatives[17], double current_state[17], double motors_duty_cycle[4]) {
    double Cb = current_state[0];
    double omega1 = current_state[1];
    double omega2 = current_state[2];
    double omega3 = current_state[3];
    double omega4 = current_state[4];
    double phi = current_state[5];
    double theta = current_state[6];
    double psi = current_state[7];
    //double x = current_state[8];
    //double y = current_state[9];
    //double z = current_state[10];
    double dphi = current_state[11];
    double dtheta = current_state[12];
    double dpsi = current_state[13];
    double dx = current_state[14];
    double dy = current_state[15];
    double dz = current_state[16];

    double m1_duty_cycle = motors_duty_cycle[0];
    double m2_duty_cycle = motors_duty_cycle[1];
    double m3_duty_cycle = motors_duty_cycle[2];
    double m4_duty_cycle = motors_duty_cycle[3];

    double Vbattery = battery_voltage(Cb/battery_capacity);         //battery voltage (V)
    double Im1 = motor_current(omega1, m1_duty_cycle, Vbattery);        //current on motor #1 (A)
    double Im2 = motor_current(omega2, m2_duty_cycle, Vbattery);        //current on motor #2 (A)
    double Im3 = motor_current(omega3, m3_duty_cycle, Vbattery);        //current on motor #3 (A)
    double Im4 = motor_current(omega4, m4_duty_cycle, Vbattery);        //current on motor #4 (A)
    double Ielectronics = 2.5 / Vbattery;                           //current flowing on the electronics (A) - equivalent to 2.5W
    double T1 = propeller_thrust(omega1);                               //rotor #1 thrust (N)
    double T2 = propeller_thrust(omega2);                               //rotor #2 thrust (N)
    double T3 = propeller_thrust(omega3);                               //rotor #3 thrust (N)
    double T4 = propeller_thrust(omega4);                               //rotor #4 thrust (N)
    double τ1 = motor_torque(omega1,m1_duty_cycle,Vbattery) - propeller_torque_hovering(omega1) - friction_torque(omega1);      //resultant torque on rotor #1 (N-m)
    double τ2 = motor_torque(omega2,m2_duty_cycle,Vbattery) - propeller_torque_hovering(omega2) - friction_torque(omega2);      //resultant torque on rotor #2 (N-m)
    double τ3 = motor_torque(omega3,m3_duty_cycle,Vbattery) - propeller_torque_hovering(omega3) - friction_torque(omega3);      //resultant torque on rotor #3 (N-m)
    double τ4 = motor_torque(omega4,m4_duty_cycle,Vbattery) - propeller_torque_hovering(omega4) - friction_torque(omega4);      //resultant torque on rotor #4 (N-m)

    double speed_vector[] = {-dx, -dy, -dz};
    double Fdrag[3];
    aerodynamic_drag_force(Fdrag, speed_vector);                    //aerodynamic drag force (N)

    derivatives[0] = 0.0 - Im1 - Im2 - Im3 - Im4 - Ielectronics;
    derivatives[1] = τ1 / Jprop;
    derivatives[2] = τ2 / Jprop;
    derivatives[3] = τ3 / Jprop;
    derivatives[4] = τ4 / Jprop;
    derivatives[5] = dphi;
    derivatives[6] = dtheta;
    derivatives[7] = dpsi;
    derivatives[8] = dx;
    derivatives[9] = dy;
    derivatives[10] = dz;
    derivatives[11] = (larm*(-T2+T4) + (Iyy-Izz)*dtheta*dpsi + Jprop*dtheta*(-omega1+omega2-omega3+omega4)) / Ixx;
    derivatives[12] = (larm*(T1-T3) + (Izz-Ixx)*dphi*dpsi + Jprop*dphi*(omega1-omega2+omega3-omega4)) / Iyy;
    derivatives[13] = (-τ1+τ2-τ3+τ4 + (Ixx-Iyy)*dtheta*dphi + Jprop*(-omega1+omega2-omega3+omega4)) / Izz;
    derivatives[14] = ((T1+T2+T3+T4)*(sin(phi)*sin(psi)+cos(psi)*sin(theta)*cos(phi)) + Fdrag[0]) / m;
    derivatives[15] = ((T1+T2+T3+T4)*(-sin(phi)*cos(psi)+sin(psi)*sin(theta)*cos(phi)) + Fdrag[1]) / m;
    derivatives[16] = (-m*g + (T1+T2+T3+T4)*cos(psi)*cos(phi) + Fdrag[2]) / m;
}


//propagate the state through time using RK4
//PROPAGATE_STATE uses the Runge-Kutta 4 method to propagate the state through
//time for a single timestep deltat
double k1[17];
double k2[17];
double k3[17];
double k4[17];
double temp_state[17];
void propagate_state(double next_state[17], double current_state[17], double motors_duty_cycle[4], double dt) {
    //calculate k1
    rhs(k1, current_state, motors_duty_cycle);

    //calculate k2
    for (int i=0; i<17; ++i) {
        temp_state[i] = current_state[i] + k1[i] * dt / 2;
    }
    rhs(k2, temp_state, motors_duty_cycle);

    //calculate k3
    for (int i=0; i<17; ++i) {
        temp_state[i] = current_state[i] + k2[i] * dt / 2;
    }
    rhs(k3, temp_state, motors_duty_cycle);

    //calculate k4
    for (int i=0; i<17; ++i) {
        temp_state[i] = current_state[i] + k3[i] * dt;
    }
    rhs(k4, temp_state, motors_duty_cycle);

    //calculate the next state
    for (int i=0; i<17; ++i) {
        next_state[i] = current_state[i] + (k1[i] + k2[i] + k3[i] + k4[i]) * dt / 6;
    }
}
