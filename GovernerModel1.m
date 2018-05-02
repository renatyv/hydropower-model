classdef GovernerModel1
    properties(Constant)
        omega_m_nom = 14.954;
        omega_ref = GovernerModel1.omega_m_nom+0.35;
        omega_dead_zone = 5*GovernerModel1.omega_m_nom*10^-5;
        omega_gov_ref = GovernerModel1.omega_m_nom+0.35;
        Pilot_max = 120;
        Pilot_min = 0;
        Pilot_base = GovernerModel1.Pilot_max;
        T_mainservo = 0.76;
        T_pilotservo = GovernerModel1.T_mainservo*1.3;
        
        PID_Kp=30;
        PID_Ki=2;
        K_f = 0.7;
%         K_dOmega must be small, otherwise unstable
        K_dOmega=0.0;
    end
    methods
        function [gov_state0] = steady(gov_params,g0)
            pilot_servo1 = g0;
            PID_i1 = g0/gov_params.PID_Ki;
            gov_state0 = [PID_i1;pilot_servo1];
        end
        
        function [dg,dgoverner_state] = model(gov_params,g,governer_state,omega_m,dOmega_m,enable_saturation,use_dead_zone)
        %governerModel implements governer model
        %   enable_saturation=true enables servo motors saturation
        %   use_dead_zone=true enable input frequency dead zone
        %   g --- gate position, p.u.
        %   PID_i --- PID integrator state
        %   pilot_servo --- pilot servo state, p.u.
        %   omega_m --- rotor frequency, rad/s
        %   dOmega_m --- rotor frequency derivative rad/s^2
        global constant_governer...
            G_base G_max G_min;

        if constant_governer
            dg = 0;
            dPID_i = 0;
            dpilot_servo = 0;
            dgoverner_state = [dPID_i;dpilot_servo];
        else
            PID_i = governer_state(1);
            pilot_servo = governer_state(2);
            omega_delta = gov_params.omega_ref-omega_m;
            if use_dead_zone
                omega_delta = dead_zone(omega_delta,...
                    -gov_params.omega_dead_zone/2,...
                    gov_params.omega_dead_zone/2);
            end
            dOmega_deadzoned = dOmega_m;
            if use_dead_zone && abs(omega_delta)<gov_params.omega_dead_zone/2
                dOmega_deadzoned = 0;
            end
            PI_in = omega_delta...
                +gov_params.K_dOmega*dOmega_deadzoned...
                -gov_params.K_f*g;

            dPID_i = PI_in;

            PID_out = gov_params.PID_Kp*PI_in...
                +gov_params.PID_Ki*PID_i;

            pilot_in = PID_out;
            pilot_max = gov_params.Pilot_max/gov_params.Pilot_base;
            pilot_min = gov_params.Pilot_min/gov_params.Pilot_base;
%             compute g_max,g_min in p.u.
            g_max = G_max/G_base;
            g_min = G_min/G_base;
            [dpilot_servo] = servoModel(pilot_in,pilot_servo,pilot_min,pilot_max,gov_params.T_pilotservo,enable_saturation);
            [dg] = servoModel(pilot_servo,g,g_min,g_max,gov_params.T_mainservo,enable_saturation);
            dgoverner_state = [dPID_i;dpilot_servo];
        end
        end
    end
end