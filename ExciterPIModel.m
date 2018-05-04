classdef ExciterPIModel
    properties(Constant)
        exciter_PID_Ki=2;
        exciter_PID_Kp=10;
    end
    properties
        use_constant_exciter=false;
        e_r_const = 0;
    end
    methods
        function [exciter_state] = steady(this,e_r)
            exciter_state = e_r;
        end
        
        function [e_r,dexciter_state] = model(this,v_q,v_d,exciter_state,enable_saturation)
        %exciterModelPI PI model of exciter governer with saturation
        v_emin = -5;
        v_emax = 5;
        v_rmin = -1;
        v_rmax = 1;
        if this.use_constant_exciter
            e_r = this.e_r_const;
            dexciter_state = 0;
        else
            v_error = 1-sqrt(v_q^2+v_d^2);
            if enable_saturation
                v_error = sat(v_error,v_rmin,v_rmax);
            end

            dexciter_state = this.exciter_PID_Ki*v_error;
            if enable_saturation && (dexciter_state>0 && exciter_state>=v_emax) || (dexciter_state<0 && exciter_state<=v_emin)
                dexciter_state = 0;
            end
            e_r = this.exciter_PID_Kp*v_error + exciter_state;
        end
        end
    end
end