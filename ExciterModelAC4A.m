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
        v_ref = 1
        v_s = 0.007;
        K_C = 0;
        T_r = 20e-3;
        state_size = 2;
    end
    
    methods
        
        function [v_ampl] = steadyV_ampl(this,e_r)
            v_ampl = this.v_ref+this.v_s-(e_r/this.K_A);
%             e_r = this.K_A*(this.v_ref-v_ampl +this.v_s);
        end
        
        function [e_r] = steadyE_r(this,v_ampl)
            e_r = this.K_A*(this.v_ref-v_ampl +this.v_s);
        end
        
        function [exciter_state] = steady(this,e_r)
        %   steady state is actually independent of e_r
        %   e_r = servo_state = servo_in
        %   servo_in = K_A*leadlag_out
        %   leadlag_in = leadlag_state = leadlag_out = e_r/K_A
        %   leadlag_in = v_error = v_ref-v_ampl+v_s
        %   v_ref-v_ampl+v_s = e_r/K_A
        %   v_ampl = v_ref+v_s-e_r/K_A
            exciter_state = [e_r/this.K_A;e_r];
        end
        
        function [e_r,dexciter_state] = model(this,v_ampl,exciter_state,enable_saturation)
            %UNTITLED IEEE AC4A exciter model
            %   Detailed explanation goes here
            dexciter_state = zeros(2,1);
            v_error = this.v_ref-v_ampl+this.v_s;
            if enable_saturation
                v_error = sat(v_error,this.v_imin,this.v_imax);
            end
            [leadlag_out,dexciter_state(1)] = leadLagFilter(v_error,exciter_state(1),this.T_C,this.T_B);
            servo_in = this.K_A*leadlag_out;
            dexciter_state(2) = servoModel(servo_in,exciter_state(2),this.v_rmin,this.v_rmax,this.T_A,enable_saturation);
            e_r = exciter_state(2);
        end
    end
end