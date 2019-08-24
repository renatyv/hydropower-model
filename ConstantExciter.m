classdef ConstantExciter
    properties(Constant)
        state_size = 0;
    end
    
    properties
        e_r_const = 1.4128;
    end
    
    methods
        function [v_ampl] = steadyV_ampl(this,e_r)
            v_ampl = 1.0;
        end
        
        function [exciter_state] = steady(this,e_r)
            exciter_state = [];
        end
        
        function [e_r,dexciter_state] = model(this,v_q,v_d,exciter_state,enable_saturation)
            %Constant exciter model
            dexciter_state = [];
            e_r = this.e_r_const;
        end
    end
end