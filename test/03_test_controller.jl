#===============================================================================
    Testing program for the proportional controller
    
    Author: Andrea Pavan
    License: MIT
===============================================================================#
using LinearAlgebra;
using Plots;

#user-defined variables
larm = 0.2250;                  #motor distance from CG (m)
Ixx = 8.825e-3;                 #body moment of inertia around x-axis (kg*m^2)
Iyy = 8.825e-3;                 #body moment of inertia around y-axis (kg*m^2)
Izz = 14.39e-3;                 #body moment of inertia around z-axis (kg*m^2)
m = 1.4750;                     #total mass (kg)
g = 9.81;                       #gravity acceleration (m/s^2)
MAX_RP_RATE = deg2rad(75);      #maximum roll/pitch rate (rad/s)
MAX_Y_RATE = deg2rad(75);       #maximum yaw rate (rad/s)
Kpϕ = Kpθ = 3.0;                #P constant on roll/pitch
Kpψ = 6.0;                      #P constant on yaw
Jprop = 10.4e-5;                #propeller moment of inertia (kg*m^2)
battery_capacity = 5000 * 3.6;  #battery capacity (coulomb) - equivalent to 5000mAh
Rdist = 0.0506972;              #power distribution resistance (Ω)
motor = (
    Kv = 1 / (920 * pi/30),     #speed constant (V/(rad/s))
    I0 = 0.5,                   #no-load current (A)
    Rm = 0.142                  #armature resistance (Ω)
);
ρ  = 1.225;                     #air density (kg/m^3)
μ = 1.81e-5;                    #air dynamic viscosity (Pa*s)
Aref = 0.1;                     #aerodynamic reference area (m2)
Δt = 0.001;                     #simulation timestep (s)
tf = 10.0;                      #simulation end time (s)

#define mixing matrix for a symmetric quadcopter
M = [ 0 -1  0  1;
      1  0 -1  0;
     -1  1 -1  1;
      1  1  1  1];
Minv = inv(M);
#Minv = [0.0   0.5  -0.25  0.25;
#       -0.5   0.0   0.25  0.25;
#        0.0  -0.5  -0.25  0.25;
#        0.5   0.0   0.25  0.25];

#proportional controller for acro/roll mode
function controller_p_acro(sticks, gyro)
    #calculate desired rates from the sticks
    ∂ϕtarget = -MAX_RP_RATE + 2*MAX_RP_RATE*sticks[1];      #desired roll rate (rad/s)
    ∂θtarget = -MAX_RP_RATE + 2*MAX_RP_RATE*sticks[2];      #desired pitch rate (rad/s)
    #Ttarget = m*g*(sticks[3]+0.5);                          #desired thrust (N)
    Ttarget = m*g*(0.2 + 1.6*sticks[3]);
    ∂ψtarget = -MAX_Y_RATE + 2*MAX_Y_RATE*sticks[4];        #desired yaw rate (rad/s)

    #proportional controller
    uϕ = Kpϕ * (∂ϕtarget - gyro[1]);
    uθ = Kpθ * (∂θtarget - gyro[2]);
    uψ = Kpψ * (∂ψtarget - gyro[3]);

    #calculate motors speed
    #=duty_cycle = sticks[3] .+ Minv * [uϕ, uθ, uψ, 0];
    for i in 1:lastindex(duty_cycle)
        duty_cycle[i] = max(0.2, duty_cycle[i]);
        duty_cycle[i] = min(1.0, duty_cycle[i]);
    end
    return duty_cycle;=#

    #SMALL EXPERIMENT: linearized model
    Kt = 8.634692591183668e-6;      #thrust-speed coefficient (N/(rad/s)^2)
    Kτ = 1.106027026514568e-7;      #torque-speed coefficient (N-m/(rad/s)^2)
    Kω = 919.2300104403735;         #speed-throttle coefficient (rad/s)
    duty_cycle = Minv * [Ixx*uϕ/(larm*Kt), Iyy*uθ/(larm*Kt), Izz*uψ/Kτ, Ttarget/Kt];
    for i in 1:lastindex(duty_cycle)
        duty_cycle[i] = max(0, duty_cycle[i]);
        duty_cycle[i] = sqrt(duty_cycle[i]) / Kω;
        duty_cycle[i] = max(0.2, duty_cycle[i]);
        duty_cycle[i] = min(1.0, duty_cycle[i]);
    end
    return duty_cycle;
end

#linear interpolation
function interp1(x, y, xq)
    if xq < x[1]
        #query point is lower than the minimum value
        return y[1];
    elseif xq > x[end]
        #query point is higher than the maximum value
        return y[end];
    else
        #query point is within the interpolating interval
        for j in 1:lastindex(x)-1
            if x[j] <= xq <= x[j+1]
                return y[j] + (xq - x[j]) * (y[j+1] - y[j]) / (x[j+1] - x[j]);
            end
        end
    end
end

#calculate motor current
function motor_current(speed, duty_cycle, source_voltage)
    Vavg = source_voltage * duty_cycle;             #average voltage (V)
    Vemf = (speed+1e-6) * motor.Kv;                 #back-emf voltage (V)
    I = (Vavg - Vemf) / (motor.Rm + Rdist);         #current (A)
    return I;
end

#define motor model
function motor_torque(speed, duty_cycle, source_voltage)
    Vavg = source_voltage * duty_cycle;             #average voltage (V)
    Vemf = (speed+1e-6) * motor.Kv;                 #back-emf voltage (V)
    I = (Vavg - Vemf) / (motor.Rm + Rdist);         #current (A)
    Vm1 = Vavg - Rdist * I;
    Pheat = motor.Rm * I^2 + Vemf * motor.I0;       #heat losses (W)
    Pin = I * Vm1;                                  #electrical power (W)
    Pout = Pin - Pheat;                             #mechanical power (W)
    τ = Pout / (speed+1e-6);                        #motor torque (N-m)
    return τ;
end

#define propeller model for torque
function propeller_torque_hovering(speed)
    return interp1(
        [0.0, 104.71975511965977, 209.43951023931953, 314.1592653589793, 418.87902047863906, 523.5987755982989, 628.3185307179587, 733.0382858376183, 837.7580409572781, 942.477796076938, 1047.1975511965977],
        [0.0, 0.0019671550966158264, 0.006445775195221762, 0.013165296892561582, 0.022061262436683073, 0.03311123098061027, 0.046312496890310625, 0.06168982012764514, 0.07927706659086173, 0.0991216985576324, 0.12128943565124396],
        speed
    );
end

#define propeller model for thrust
function propeller_thrust(speed)
    return interp1(
        [0.0, 104.71975511965977, 209.43951023931953, 314.1592653589793, 418.87902047863906, 523.5987755982989, 628.3185307179587, 733.0382858376183, 837.7580409572781, 942.477796076938, 1047.1975511965977],
        [0.0, 0.092, 0.369,  0.832,  1.484,  2.325,  3.358,  4.585,  6.010,  7.636,  9.469],
        speed
    );
end

#define friction model
function friction_torque(speed)
    #use a constant value from the first speed, with a linear ramp up to 1000rpm
    return 0.0140347 * min(1, speed/104.71975511965977);
end

#define voltage model for a 3S LiPo battery
function battery_voltage(SoC)
    #use simplified voltage chart from https://blog.ampow.com/lipo-voltage-chart/
    return interp1(
        [0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8, 0.85, 0.9, 0.95, 1],
        [9.82, 10.83, 11.06, 11.12, 11.18, 11.24, 11.3, 11.36, 11.39, 11.45, 11.51, 11.56, 11.62, 11.74, 11.86, 11.95, 12.07, 12.25, 12.33, 12.45, 12.6],
        SoC
    );
end

#define aerodynamic drag model
function aerodynamic_drag_force(speed_vector)
    Uinf = sqrt(sum(speed_vector.^2));      #airspeed magnitude (m/s)
    Re = ρ * sqrt(Aref/pi) * Uinf / μ;      #reynolds number
    Re += 1.0;
    CD = 0.40;                              #drag coefficient (turbulent separation)
    if Re < 350_000
        #use R22 formula without correction
        #to estimate the drag coefficient with laminar separation
        CD = (1.0 + (Re*3/16) / (1.0 + (Re*19/240) / (1.0 + Re*1/122))) * 24/Re
    end
    return 0.5 * ρ * speed_vector * Uinf * Aref * CD;
end

#define ODE system of equations
function rhs(current_state, motors_duty_cycle)
    (Cb,ω1,ω2,ω3,ω4,ϕ,θ,ψ,x,y,z,∂ϕ,∂θ,∂ψ,∂x,∂y,∂z) = current_state;
    (m1_duty_cycle, m2_duty_cycle, m3_duty_cycle, m4_duty_cycle) = motors_duty_cycle;
    Vbattery = battery_voltage(Cb/battery_capacity);        #battery voltage (V)
    Im1 = motor_current(ω1, m1_duty_cycle, Vbattery);       #current on motor #1 (A)
    Im2 = motor_current(ω2, m2_duty_cycle, Vbattery);       #current on motor #2 (A)
    Im3 = motor_current(ω3, m3_duty_cycle, Vbattery);       #current on motor #3 (A)
    Im4 = motor_current(ω4, m4_duty_cycle, Vbattery);       #current on motor #4 (A)
    Ielectronics = 2.5 / Vbattery;                          #current flowing on the electronics (A) - equivalent to 2.5W
    T1 = propeller_thrust(ω1);                              #rotor #1 thrust (N)
    T2 = propeller_thrust(ω2);                              #rotor #2 thrust (N)
    T3 = propeller_thrust(ω3);                              #rotor #3 thrust (N)
    T4 = propeller_thrust(ω4);                              #rotor #4 thrust (N)
    τ1 = motor_torque(ω1,m1_duty_cycle,Vbattery) - propeller_torque_hovering(ω1) - friction_torque(ω1);        #resultant torque on rotor #1 (N-m)
    τ2 = motor_torque(ω2,m2_duty_cycle,Vbattery) - propeller_torque_hovering(ω2) - friction_torque(ω2);        #resultant torque on rotor #2 (N-m)
    τ3 = motor_torque(ω3,m3_duty_cycle,Vbattery) - propeller_torque_hovering(ω3) - friction_torque(ω3);        #resultant torque on rotor #3 (N-m)
    τ4 = motor_torque(ω4,m4_duty_cycle,Vbattery) - propeller_torque_hovering(ω4) - friction_torque(ω4);        #resultant torque on rotor #4 (N-m)
    Fdrag = aerodynamic_drag_force(-[∂x,∂y,∂z]);            #aerodynamic drag force (N)
    return [
        0.0 - Im1 - Im2 - Im3 - Im4 - Ielectronics,
        τ1 / Jprop,
        τ2 / Jprop,
        τ3 / Jprop,
        τ4 / Jprop,
        ∂ϕ,
        ∂θ,
        ∂ψ,
        ∂x,
        ∂y,
        ∂z,
        (larm*(-T2+T4) + (Iyy-Izz)*∂θ*∂ψ + Jprop*∂θ*(-ω1+ω2-ω3+ω4)) / Ixx,
        (larm*(T1-T3) + (Izz-Ixx)*∂ϕ*∂ψ + Jprop*∂ϕ*(ω1-ω2+ω3-ω4)) / Iyy,
        (-τ1+τ2-τ3+τ4 + (Ixx-Iyy)*∂θ*∂ϕ + Jprop*(-ω1+ω2-ω3+ω4)) / Izz,
        ((T1+T2+T3+T4)*(sin(ϕ)*sin(ψ)+cos(ψ)*sin(θ)*cos(ϕ)) + Fdrag[1]) / m,
        ((T1+T2+T3+T4)*(-sin(ϕ)*cos(ψ)+sin(ψ)*sin(θ)*cos(ϕ)) + Fdrag[2]) / m,
        (-m*g + (T1+T2+T3+T4)*cos(ψ)*cos(ϕ) + Fdrag[3]) / m
    ];
end

#time marching using explicit RK4 method
t = collect(0:Δt:tf);           #time vector (s)
state = zeros(17,length(t));    #state vector evolving over time
state[:,1] = [                              #initial conditions
    battery_capacity,           #battery charge Cb (coulomb)
    0.0,                        #motor #1 speed ω1 (rad/s)
    0.0,                        #motor #2 speed ω2 (rad/s)
    0.0,                        #motor #3 speed ω3 (rad/s)
    0.0,                        #motor #4 speed ω4 (rad/s)
    0.0,                        #roll angle ϕ (rad)
    0.0,                        #pitch angle θ (rad)
    0.0,                        #yaw angle ψ (rad)
    0.0,                        #position x (m)
    0.0,                        #position y (m)
    0.0,                        #altitude z (m)
    0.0,                        #roll rate ∂ϕ/∂t (rad/s)
    0.0,                        #pitch rate ∂θ/∂t (rad/s)
    0.0,                        #yaw rate ∂ψ/∂t (rad/s)
    0.0,                        #horizontal speed Ux (m/s)
    0.0,                        #horizontal speed Uy (m/s)
    0.0                         #vertical speed Uz (m/s)
];
pilot_input = zeros(4,length(t));           #stick positions over time
pilot_input .= 0.5;
pilot_input[3,1:1500] .= 0.7;       #takeoff
pilot_input[3,1501:3000] .= 0.2;
pilot_input[3,3001:3500] .= 0.3;
pilot_input[3,3501:4000] .= 0.4;
pilot_input[3,4001:5000] .= 0.57;
pilot_input[3,5001:6000] .= 0.75;
pilot_input[3,6001:7000] .= 0.8;
pilot_input[3,7001:8000] .= 0.51;
pilot_input[3,8001:end] .= 0.4;
pilot_input[4,3000:5000] .= 0.7;    #yaw rate to 30deg/s
pilot_input[4,5001:7000] .= 0.3;
pilot_input[1,3000:6000] .= 0.7;    #roll rate to 30deg/s
pilot_input[1,6001:9000] .= 0.3;
motors_pwm = zeros(4,length(t));    #duty cycle of each motor over time
motors_pwm[:,1] = controller_p_acro(pilot_input[:,1], state[12:14,1]);
println("z(0) = ", state[11,1], " m");
time0 = time();
for i in 2:lastindex(t)
    #explicit RK4 method
    #assumption: the pilot input is constant over a timestep
    k1 = rhs(state[:,i-1], motors_pwm[:,i-1]);
    k2 = rhs(state[:,i-1] + k1*Δt/2, motors_pwm[:,i-1]);
    k3 = rhs(state[:,i-1] + k2*Δt/2, motors_pwm[:,i-1]);
    k4 = rhs(state[:,i-1] + k3*Δt, motors_pwm[:,i-1]);
    state[:,i] = state[:,i-1] + (k1 + k2 + k3 + k4) * Δt/6;

    #fully-discharged battery
    if state[1,i] <= 0
        state[1,i] = 0;     #Cb=0
    end

    #on the ground
    if state[11,i] <= 0
        state[11,i] = 0;    #z=0
        state[17,i] = 0;    #∂z=0
    end

    #controller
    motors_pwm[:,i] = motors_pwm[:,i-1];
    if i%4 == 0
        #update control at 250Hz
        motors_pwm[:,i] = controller_p_acro(pilot_input[:,i], state[12:14,i]);
    end

    #print results every 0.1 seconds
    if i%100 == 0
        println("z(",t[i],") = ", state[11,i], " m");
    end
end
println("Completed in ", round(time()-time0,digits=2), " s");

#plot altitude profile
plt1 = plot(t, state[11,:],
    title = "Altitude profile",
    legend = false,
    xlabel = "Time (s)",
    ylabel = "Altitude (m)"
);
display(plt1);
