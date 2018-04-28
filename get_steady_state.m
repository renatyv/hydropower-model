function [steady_state_1,steady_state_2,N_turb,e_r_1,e_r_2] =...
    get_steady_state(P_active0,Q_reactive0)
%get_steady_state computes power plant state
%   ex
global poles_number S_base r_s...
    k_feedback constant_governer PID_Ki...
    omega_gov_ref;
%% generator steady torque
phi_1 = atan(Q_reactive0/P_active0);
i_ampl = P_active0/(S_base*cos(phi_1));
N_gen = -((i_ampl^2)*r_s*S_base+P_active0);
N_turb = -N_gen;

%% turbine and vogerner steady state
omega_of_g = @(g)(omega_gov_ref - k_feedback*g);
if constant_governer
    omega_of_g = @(g)omega_gov_ref;
end
[g1, q1]=turbineSteadyState(N_turb,omega_of_g);

omega_m0 = omega_of_g(g1);
omega_er0 = omega_m0*poles_number;

%% governer steady state
pilot_servo1 = g1;
PID_i1 = g1/PID_Ki;

hydro_state_1 = [q1; g1; PID_i1;pilot_servo1];


[psi_1,e_r_1,psi_2,e_r_2] =...
    generatorSteadyStateDan(i_ampl, phi_1,omega_er0);

%% exciter steady state
exciter_state_1  = e_r_1;
exciter_state_2  = e_r_2;

%% complete steady state
steady_state_1 = [omega_m0;hydro_state_1;psi_1;exciter_state_1];
steady_state_2 = [omega_m0;hydro_state_1;psi_2;exciter_state_2];
end

