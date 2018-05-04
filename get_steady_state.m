function [steady_state_1,steady_state_2,N_turb,e_r_1,e_r_2] =...
    get_steady_state(gov_model,gen_model,turb_model,exciter_model,load_model)
%get_steady_state computes power plant state
%   ex
%% generator steady torque
P0 = load_model.P_active0;
S_base = gen_model.S_base;
phi_1 = load_model.phi_1;
r_s = gen_model.r_s;
i_ampl = P0/(S_base*cos(phi_1));
N_gen = -((i_ampl^2)*r_s*S_base+P0);
N_turb = -N_gen;

%% turbine and vogerner steady state
omega_ref = gov_model.omega_ref;
K_f = gov_model.K_f;
omega_of_g = @(g)(omega_ref - K_f*g);
if gov_model.constant_governer
    omega_of_g = @(g)omega_ref;
end
[g0, q0]=turb_model.steady(N_turb,omega_of_g);

omega_m0 = omega_of_g(g0);
omega_er0 = omega_m0*gen_model.poles_number;

%% governer steady state
gov_state0 = gov_model.steady(g0);

%% generator steady states
[psi_1,e_r_1,psi_2,e_r_2] =...
    gen_model.steady(i_ampl, phi_1,omega_er0);

%% exciter steady state
exciter_state_1  = exciter_model.steady(e_r_1);
exciter_state_2  = exciter_model.steady(e_r_2);

%% complete steady state
steady_state_1 = constructState(omega_m0,q0,g0,gov_state0,psi_1,exciter_state_1);
steady_state_2 = constructState(omega_m0,q0,g0,gov_state0,psi_2,exciter_state_2);
end

