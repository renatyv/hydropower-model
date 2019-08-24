classdef ConstantGoverner
    properties (Constant)
        state_size = 0;
        omega_ref = 14.954;
        K_f = 0;
    end
    methods
        
        function [omega_m] = steady_freq(this,g)
            omega_m = this.omega_ref - this.K_f*g;
        end
        
        function [gov_state0] = steady(this,g0)
            gov_state0 = [];
        end
        
        function [dg,dgoverner_state] = model(this,g,governer_state,omega_m,dOmega_m,enable_saturation,use_dead_zone)
        dg = 0;
        dgoverner_state = [];
        end
    end
end

