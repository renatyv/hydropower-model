function [dg,dPID_i,dpilot_servo] = governerModel(g,PID_i,pilot_servo,omega_m,dOmega_m)
%REGULATOR_MODEL Summary of this function goes here
%   Detailed explanation goes here
global PID_Kp PID_Ki PID_Kd...
    omega_dead_zone omega_m_nom...
    G_max G_min G_base...
    T_pilotservo T_mainservo...
    Pilot_max Pilot_min Pilot_base...
    constant_governer use_dead_zone...
    k_feedback use_integrator use_pilot_servo;

if constant_governer
    dg = 0;
    dPID_i = 0;
    dpilot_servo = 0;
else
    omega_delta = omega_m_nom-omega_m;
    if use_dead_zone
        omega_delta = dead_zone(omega_delta,-omega_dead_zone/2,omega_dead_zone/2);
    end
    PID_in = omega_delta - k_feedback*g;
    
    dPID_i = PID_in;
    PID_d = dOmega_m;
    if use_dead_zone && abs(omega_delta)<omega_dead_zone/2
        PID_d = 0;
    end
    PID_out = PID_Kp*PID_in...
        +PID_Ki*PID_i...
        +PID_Kd*PID_d;
    
    pilot_in = PID_out;
    pilot_max = Pilot_max/Pilot_base;
    pilot_min = Pilot_min/Pilot_base;
    g_max = G_max/G_base;
    g_min = G_min/G_base;
    if use_pilot_servo
        [dpilot_servo] = servoModel(pilot_in,pilot_servo,pilot_min,pilot_max,T_pilotservo);
        [dg] = servoModel(pilot_servo,g,g_min,g_max,T_mainservo);
    else
        dpilot_servo = 0;
        [dg] = servoModel(PID_out,g,g_min,g_max,T_mainservo);
    end
end
end

