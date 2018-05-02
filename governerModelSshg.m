function [dg,dGov] = governerModelSshg(g,gov,omega_m,dOmega_m)
%REGULATOR_MODEL Summary of this function goes here
%   Detailed explanation goes here
global PID_Kp PID_Ki K_dOmega...
    omega_dead_zone...
    G_max G_min G_base...
    constant_governer use_dead_zone...
    K_f omega_gov_ref;
 
 
T_mainservo = 0.7;
T_pilotservo = T_mainservo;
T_driveservo = T_mainservo*1.3;
[drive_servo,pilot_servo,PID_i] = parseGov(gov);
if constant_governer
    dg = 0;
    dPID_i = 0;
    dpilot_servo = 0;
    ddrive_servo = 0;
else
    omega_delta = omega_gov_ref-omega_m;
    if use_dead_zone
        omega_delta = dead_zone(omega_delta,-omega_dead_zone/2,omega_dead_zone/2);
    end
    dOmega_deadzoned = dOmega_m;
    if use_dead_zone && abs(omega_delta)<omega_dead_zone/2
        dOmega_deadzoned = 0;
    end
    PI_in = omega_delta +K_dOmega*dOmega_deadzoned - K_f*drive_servo;
    
    dPID_i = PI_in;
    
    PID_out = PID_Kp*PI_in...
        +PID_Ki*PID_i;
    
    [ddrive_servo] = servoModel(PID_out,drive_servo,0,1,T_driveservo);
    drive_servo_in = drive_servo-pilot_servo-g;
    [dpilot_servo] = servoModel(drive_servo_in,pilot_servo,-1,1,T_pilotservo);
    [dg] = servoModel(pilot_servo,g,0,1,T_mainservo);
end
dGov = [ddrive_servo,dpilot_servo,dPID_i];
end