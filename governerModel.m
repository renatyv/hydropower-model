function [dg,dgoverner_state] = governerModel(g,governer_state,omega_m,dOmega_m,enable_saturation,use_dead_zone)
%governerModel implements governer model
%   enable_saturation=true enables servo motors saturation
%   use_dead_zone=true enable input frequency dead zone
%   g --- gate position, p.u.
%   PID_i --- PID integrator state
%   pilot_servo --- pilot servo state, p.u.
%   omega_m --- rotor frequency, rad/s
%   dOmega_m --- rotor frequency derivative rad/s^2
global PID_Kp PID_Ki K_dOmega...
    omega_dead_zone...
    G_max G_min G_base...
    T_pilotservo T_mainservo...
    Pilot_max Pilot_min Pilot_base...
    constant_governer...
    k_feedback omega_gov_ref;

if constant_governer
    dg = 0;
    dPID_i = 0;
    dpilot_servo = 0;
else
    PID_i = governer_state(1);
    pilot_servo = governer_state(2);
    omega_delta = omega_gov_ref-omega_m;
    if use_dead_zone
        omega_delta = dead_zone(omega_delta,-omega_dead_zone/2,omega_dead_zone/2);
    end
    dOmega_deadzoned = dOmega_m;
    if use_dead_zone && abs(omega_delta)<omega_dead_zone/2
        dOmega_deadzoned = 0;
    end
    PI_in = omega_delta +K_dOmega*dOmega_deadzoned - k_feedback*g;
    
    dPID_i = PI_in;
    
    PID_out = PID_Kp*PI_in...
        +PID_Ki*PID_i;
    
    pilot_in = PID_out;
    pilot_max = Pilot_max/Pilot_base;
    pilot_min = Pilot_min/Pilot_base;
    g_max = G_max/G_base;
    g_min = G_min/G_base;
    [dpilot_servo] = servoModel(pilot_in,pilot_servo,pilot_min,pilot_max,T_pilotservo,enable_saturation);
    [dg] = servoModel(pilot_servo,g,g_min,g_max,T_mainservo,enable_saturation);
    dgoverner_state = [dPID_i;dpilot_servo];
end
end

