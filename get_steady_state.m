function [steady_state_1,steady_state_2,N_turb,e_r_1,e_r_2] =...
    get_steady_state(governer_params,P_active0,Q_reactive0)
%get_steady_state computes power plant state
%   ex
global poles_number S_base r_s...
    constant_governer;
%% generator steady torque
phi_1 = atan(Q_reactive0/P_active0);
i_ampl = P_active0/(S_base*cos(phi_1));
N_gen = -((i_ampl^2)*r_s*S_base+P_active0);
N_turb = -N_gen;

%% turbine and vogerner steady state
omega_of_g = @(g)(governer_params.omega_ref - governer_params.K_f*g);
if constant_governer
    omega_of_g = @(g)governer_params.omega_gov_ref;
end
[g0, q0]=turbineSteadyState(N_turb,omega_of_g);

omega_m0 = omega_of_g(g0);
omega_er0 = omega_m0*poles_number;

%% governer steady state
pilot_servo1 = g0;
PID_i1 = g0/governer_params.PID_Ki;
gov_state0 = [PID_i1;pilot_servo1];

%% generator steady states
[psi_1,e_r_1,psi_2,e_r_2] =...
    generatorSteadyState(i_ampl, phi_1,omega_er0);

%% exciter steady state
exciter_state_1  = e_r_1;
exciter_state_2  = e_r_2;

%% complete steady state
steady_state_1 = constructState(omega_m0,q0,g0,gov_state0,psi_1,exciter_state_1);
steady_state_2 = constructState(omega_m0,q0,g0,gov_state0,psi_2,exciter_state_2);
end

