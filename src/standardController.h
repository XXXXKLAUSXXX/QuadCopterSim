#ifndef controllerTarg 
#define controllerTarg "standard"

#include "./simulator.c"

//proportional controller for acro/roll mode.
//CONTROLLER_P_ACRO calculates the desired rates from the sticks and applies a
//proportional controller to calculate the motors' duty cycles
//Duty cycle for each motor?
//sticks x,y,x,y?
//Gyro XYZ rot?
void controller_p_acro(double duty_cycle[4], double sticks[4], double gyro[3], double accel[3], double altitude) {
    //calculate the desired rates from the stick positions
    double dphitarget = -MAX_RP_RATE + 2*MAX_RP_RATE*sticks[0];     //desired roll rate (rad/s)
    double dthetatarget = -MAX_RP_RATE + 2*MAX_RP_RATE*sticks[1];   //desired pitch rate (rad/s)
    double Ttarget = m*g*(0.2 + 1.6*sticks[2]);                     //desired total thrust (N)
    double dpsitarget = -MAX_Y_RATE + 2*MAX_Y_RATE*sticks[3];       //desired yaw rate (rad/s)

    //proportional controller
    double uphi = Kpphi * (dphitarget - gyro[0]);
    double utheta = Kptheta * (dthetatarget - gyro[1]);
    double upsi = Kppsi * (dpsitarget - gyro[2]);

    //calculate motors' duty cycles
    double temp[4];
    temp[0] = Ixx*uphi/(larm*Kt);
    temp[1] = Iyy*utheta/(larm*Kt);
    temp[2] = Izz*upsi/Ktau;
    temp[3] = Ttarget/Kt;

    for (int i = 0; i<4; ++i) {
        duty_cycle[i] = 0.0;
        for (int j = 0; j < 4; j++) {
            duty_cycle[i] += Minv[i][j] * temp[j];
        }
        if (duty_cycle[i] < 0.0) {
            duty_cycle[i] = 0.0;
        }
        duty_cycle[i] = sqrt(duty_cycle[i]) / Komega;
        if (duty_cycle[i] < 0.2) {
            duty_cycle[i] = 0.2;
        }
        if (duty_cycle[i] > 1.0) {
            duty_cycle[i] = 1.0;
        }
    }
}


#endif