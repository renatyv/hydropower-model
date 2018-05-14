function [steady_state_1,steady_state_2,N_turb] =...
    get_steady_state(gov_model,gen_model,turb_model,exciter_model,load_model)
%get_steady_state computes power plant state
%   ex
%% generator steady torque
P0 = load_model.P_active0;
S_base = gen_model.S_base;
phi_1 = load_model.phi_1;
r_s = gen_model.r_s;
function [error,steady_state_1,steady_state_2,e_r_1] = s_state(v_ampl)
    i_ampl = P0/(S_base*v_ampl*cos(phi_1));
    N_gen = -((i_ampl^2)*r_s*S_base+P0);
    N_turb = -N_gen;

    %% turbine and vogerner steady state
    [g0, q0]=turb_model.steady(N_turb,@gov_model.steady_freq);

    omega_m0 = gov_model.steady_freq(g0);
    omega_er0 = omega_m0*gen_model.poles_number;

    %% governer steady state
    gov_state0 = gov_model.steady(g0);

    %% generator steady states
    [psi_1,e_r_1,psi_2,e_r_2] =...
        gen_model.steady(i_ampl,v_ampl, phi_1,omega_er0);

    %% exciter steady state
    exciter_state_1  = exciter_model.steady(e_r_1);
    exciter_state_2  = exciter_model.steady(e_r_2);
    error = min(abs(exciter_model.steadyV_ampl(e_r_1)-v_ampl),abs(exciter_model.steadyV_ampl(e_r_2)-v_ampl));
% 	error = min(abs(exciter_model.steadyE_r(v_ampl)-e_r_1),abs(exciter_model.steadyE_r(v_ampl)-e_r_2));

    %% complete steady state
    steady_state_1 = constructState(omega_m0,q0,g0,gov_state0,psi_1,exciter_state_1);
    steady_state_2 = constructState(omega_m0,q0,g0,gov_state0,psi_2,exciter_state_2); 
end

function error = error_func(v_ampl)
    [error,~,~,~] = s_state(v_ampl);
end
options = optimset('MaxIter',1000,'TolFun',1e-12);
v_ampl = fminsearch(@(v)error_func(v),1.0,options)
% v_ampls = (0.9:10^-5:1.1);
% errors = -ones(size(v_ampls));
% for k=1:length(v_ampls)
%     [errors(k),~,~] = s_state(v_ampls(k));
% end
% [min_error,index]=min(errors)
% v_ampl = v_ampls(index)
[~,steady_state_1,steady_state_2,e_r_1] = s_state(v_ampl)
end