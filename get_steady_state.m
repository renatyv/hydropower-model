function [steady_state_1,steady_state_2,N_turb] =...
    get_steady_state(turb_model,gov_model,gen_model,exciter_model,load_model)
%get_steady_state computes power plant state
%   ex
%% generator steady torque
P0 = load_model.P_active0;
S_base = gen_model.S_base;
phi_1 = load_model.phi_1;
r_s = gen_model.r_s;
function [error,steady_state_1,steady_state_2,e_r_1,i_ampl] = s_state(v_ampl)
    i_ampl = load_model.get_i_ampl(v_ampl,gen_model.S_base);
    
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
    [e_r,dex]= exciter_model.model(v_ampl,exciter_state_1,false);
    error = 10*(dex'*dex)+abs(e_r-e_r_1);
%     error = min(abs(exciter_model.steadyV_ampl(e_r_1)-v_ampl),abs(exciter_model.steadyV_ampl(e_r_2)-v_ampl));
% 	error = min(abs(exciter_model.steadyE_r(v_ampl)-e_r_1),abs(exciter_model.steadyE_r(v_ampl)-e_r_2));

    %% complete steady state
    steady_state_1 = constructState(omega_m0,q0,g0,gov_state0,psi_1,exciter_state_1);
    steady_state_2 = constructState(omega_m0,q0,g0,gov_state0,psi_2,exciter_state_2); 
end

function error = error_func(v_ampl)
    [error,~,~,~] = s_state(v_ampl);
end
options = optimset('MaxIter',2000,'TolFun',1e-15,'TolX',1e-15);
% v_ampl = 1.0009
[v_ampl,final_error] = fminsearch(@(v)error_func(v),1.0,options);
% error = 1e-23, i_ampl = 0.8672
[error,steady_state_1,steady_state_2,e_r_1,i_ampl] = s_state(v_ampl);
end