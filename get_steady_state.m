function [steady_state_1,steady_state_2,N_turb,exciter_state_1,exciter_state_2] =...
    get_steady_state(z_tailrace,P_active0,Q_reactive0)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
global omega_m_nom poles_number torque_base S_base G_base Q_base r_s...
    k_feedback constant_governer PID_Ki;
% generatorParametersDAN;
% turbineGovernerParameters;
% exciterParameters;

% % % initial frequency in radian/s, electrical radian/s, HZ and rpm

%% generator steady torque
phi_1 = atan(Q_reactive0/P_active0);
i_ampl = P_active0/(S_base*cos(phi_1));
N_gen = -((i_ampl^2)*r_s*S_base+P_active0);
N_turb = -N_gen;

%% turbine and vogerner steady state
omega_of_g = @(g)(omega_m_nom - k_feedback*g);
if constant_governer
    omega_of_g = @(g)omega_m_nom;
end
[g1, q1]=turbineSteadyState(N_turb,omega_of_g,z_tailrace);

omega_m0 = omega_of_g(g1);
omega_er0 = omega_m0*poles_number;

%% governer steady state
pilot_servo1 = g1;
PID_i1 = g1/PID_Ki;

hydro_state_1 = [q1, g1, PID_i1,pilot_servo1];

% omega_m0 = omega_m_nom;
% omega_er0 = omega_m_nom*poles_number;

[gen_state_1,gen_state_2] =...
    generatorSteadyStateDan(i_ampl, phi_1,omega_er0);

% state = [psi_d,psi_q,psi_r,psi_rd,psi_rq,E_r,theta,t_shaft,phi_1]
theta_er1 = gen_state_1(6);
theta_er2 = gen_state_2(6);
% field voltage
exciter_state_1  = gen_state_1(7);
t_shaft1 = gen_state_1(8);

exciter_state_2 = gen_state_2(7);
t_shaft2 = gen_state_2(8);

% the same for both steady states
phi_1 = gen_state_1(9);


steady_state_1 = [omega_m0,hydro_state_1,gen_state_1(1:5),exciter_state_1];
steady_state_2 = [omega_m0,hydro_state_1,gen_state_2(1:5),exciter_state_2];

% isStateSteady(initial_state_1,P_active0,Q_reactive0,z_tailrace,e_r1);
% isStateSteady(initial_state_2,P_active0,Q_reactive0,z_tailrace,e_r2);
end

