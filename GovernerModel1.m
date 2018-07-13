classdef GovernerModel1
    properties(Constant)
        omega_m_nom = 14.954;
        omega_ref = GovernerModel1.omega_m_nom+0.35;
        omega_dead_zone = 5*GovernerModel1.omega_m_nom*10^-5;
        Pilot_max = 120;
        Pilot_min = 0;
        Pilot_base = GovernerModel1.Pilot_max;
        T_mainservo = 0.76;
        T_pilotservo = GovernerModel1.T_mainservo*1.3;
%         K_dOmega must be small, otherwise unstable
        K_dOmega=0.0;
        g_max = 1.0;
        g_min = 0.0;
        state_size = 2;
    end
    properties
        PID_Kp=30;
        PID_Ki=2; 
        K_f = 0.7;
    end
    
    methods
        function [omega_m] = steady_freq(this,g)
            omega_m = this.omega_ref - this.K_f*g;
        end
        
        function [gov_state0] = steady(this,g0)
            pilot_servo1 = g0;
            PID_i1 = g0/this.PID_Ki;
            gov_state0 = [PID_i1;pilot_servo1];
        end
        
        function [dg,dgoverner_state] = model(this,g,governer_state,omega_m,dOmega_m,enable_saturation,use_dead_zone)
        %governerModel implements governer model
        %   enable_saturation=true enables servo motors saturation
        %   use_dead_zone=true enable input frequency dead zone
        %   g --- gate position, p.u.
        %   PID_i --- PID integrator state
        %   pilot_servo --- pilot servo state, p.u.
        %   omega_m --- rotor frequency, rad/s
        %   dOmega_m --- rotor frequency derivative rad/s^2
        PID_i = governer_state(1);
        pilot_servo = governer_state(2);
        omega_delta = this.omega_ref-omega_m;
        if use_dead_zone
            omega_delta = dead_zone(omega_delta,...
                -this.omega_dead_zone/2,...
                this.omega_dead_zone/2);
        end
        dOmega_deadzoned = dOmega_m;
        if use_dead_zone && abs(omega_delta)<this.omega_dead_zone/2
            dOmega_deadzoned = 0;
        end
        PI_in = omega_delta...
            +this.K_dOmega*dOmega_deadzoned...
            -this.K_f*g;

        dPID_i = PI_in;

        PID_out = this.PID_Kp*PI_in...
            +this.PID_Ki*PID_i;

        pilot_in = PID_out;
        pilot_max = this.Pilot_max/this.Pilot_base;
        pilot_min = this.Pilot_min/this.Pilot_base;
%             compute g_max,g_min in p.u.
        [dpilot_servo] = servoModel(pilot_in,pilot_servo,pilot_min,pilot_max,this.T_pilotservo,enable_saturation);
        [dg] = servoModel(pilot_servo,g,this.g_min,this.g_max,this.T_mainservo,enable_saturation);
        dgoverner_state = [dPID_i;dpilot_servo];
        end
    end
end