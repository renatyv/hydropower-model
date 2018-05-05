classdef GovernerModelSSHG
    properties (Constant)
        omega_m_nom = 14.954;
        omega_ref = GovernerModelSSHG.omega_m_nom+0.35;
        omega_dead_zone = 5*GovernerModelSSHG.omega_m_nom*10^-5;
        Pilot_max = 120;
        Pilot_min = 0;
        Pilot_base = GovernerModelSSHG.Pilot_max;
        T_mainservo = 0.76;
        T_pilotservo = GovernerModelSSHG.T_mainservo*1.3;
        T_driveservo = GovernerModelSSHG.T_pilotservo;
        
        PID_Kp=30;
        PID_Ki=2;
        K_f = 0.7;
        K_dOmega=0.0;
        g_max = 1.0;
        g_min = 0.0;
        state_size = 3;
    end
    methods
        function [gov_state0] = steady(this,g0)
            drive_servo = g0;
            pilot_servo = 0;
            PID_i = 0;
            gov_state0 = [drive_servo;pilot_servo;PID_i];
        end
        
        function [dg,dgoverner_state] = model(this,g,governer_state,omega_m,dOmega_m,enable_saturation,use_dead_zone)
        %REGULATOR_MODEL Summary of this function goes here
        %   Detailed explanation goes here
        drive_servo = governer_state(1);
        pilot_servo = governer_state(2);
        PID_i = governer_state(3);
        omega_delta = this.omega_ref-omega_m;
        omega_dead_zone = GovernerModelSSHG.omega_dead_zone;
        if use_dead_zone
            omega_delta = dead_zone(omega_delta,-omega_dead_zone/2,omega_dead_zone/2);
        end
        dOmega_deadzoned = dOmega_m;
        if use_dead_zone && abs(omega_delta)<omega_dead_zone/2
            dOmega_deadzoned = 0;
        end
        PI_in = omega_delta +this.K_dOmega*dOmega_deadzoned - this.K_f*drive_servo;

        dPID_i = PI_in;

        PID_out = this.PID_Kp*PI_in...
            +this.PID_Ki*PID_i;

        ddrive_servo = saturatedIntegrator(PID_out,drive_servo,0,1,1/this.T_driveservo,enable_saturation);
        drive_servo_in = drive_servo-pilot_servo-g;
        dpilot_servo = saturatedIntegrator(drive_servo_in,pilot_servo,-1,1,1/this.T_pilotservo,enable_saturation);
        dg = saturatedIntegrator(pilot_servo,g,0,1,1/this.T_mainservo,enable_saturation);
        dgoverner_state = [ddrive_servo;dpilot_servo;dPID_i];
        end
    end
end

