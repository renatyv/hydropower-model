classdef ExciterModelAC4A
    properties(Constant)
        v_rmin = -4.53; 
        v_rmax = 5.64;
        v_imin = -10;
        v_imax = 10;
        T_A = 0.015;
        T_B = 10.0;
        T_C = 1.0;
        K_A = 200;
        v_fref = 1
        v_s = 0;
        K_C = 0;
        T_r = 20e-3;
    end
    
    methods
        function [e_r] = steadyE(this,v_in)
            e_r = this.K_A*(this.v_ref-v_in +this.v_s);
        end
        
        function [exciter_state] = steady(this,e_r)
            exciter_state(2) = e_r;
            exciter_state(1) = e_r/this.K_A;
        end
        
        function [e_r,dexciter_state] = model(this,v_q,v_d,exciter_state,enable_saturation)
        %UNTITLED IEEE AC4A exciter model
        %   Detailed explanation goes here
        dexciter_state = zeros(2,1);
        v_error = this.v_fref-sqrt(v_q^2+v_d^2)+this.v_s;
        if enable_saturation
            v_error = sat(v_error,this.v_imin,this.v_imax);
        end
        [leadlag_out,dexciter_state(1)] = leadLagFilter(v_error,exciter_state(1),this.T_C,this.T_B);
        [dexciter_state(2)] = servoModel(this.K_A*leadlag_out,exciter_state(2),this.v_rmin,this.v_rmax,this.T_A,enable_saturation);
        e_r = exciter_state(2);
        end
    end
end