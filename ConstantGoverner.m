classdef ConstantGoverner
    properties (Constant)
        state_size = 0;
        omega_ref = 14.954;
        K_f = 0;
    end
    methods
        
        function [gov_state0] = steady(this,g0)
            gov_state0 = [];
        end
        
        function [dg,dgoverner_state] = model(this,g,governer_state,omega_m,dOmega_m,enable_saturation,use_dead_zone)
        %REGULATOR_MODEL Summary of this function goes here
        %   Detailed explanation goes here
        dg = 0;
        dgoverner_state = [];
        end
    end
end

