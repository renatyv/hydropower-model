function [steady_state_1,steady_state_2,N_turb,e_r_1,e_r_2] =...
    get_steady_state(gov_model,gen_model,turb_model,P_active0,Q_reactive0)
%get_steady_state computes power plant state
%   ex
global constant_governer;
%% generator steady torque
phi_1 = atan(Q_reactive0/P_active0);
i_ampl = P_active0/(gen_model.S_base*cos(phi_1));
N_gen = -((i_ampl^2)*gen_model.r_s*gen_model.S_base+P_active0);
N_turb = -N_gen;

%% turbine and vogerner steady state
omega_of_g = @(g)(gov_model.omega_ref - gov_model.K_f*g);
if constant_governer
    omega_of_g = @(g)gov_model.omega_gov_ref;
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
exciter_state_1  = e_r_1;
exciter_state_2  = e_r_2;

%% complete steady state
steady_state_1 = constructState(omega_m0,q0,g0,gov_state0,psi_1,exciter_state_1);
steady_state_2 = constructState(omega_m0,q0,g0,gov_state0,psi_2,exciter_state_2);
end

