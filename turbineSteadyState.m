function [g0, q0, z_tailrace]=...
    turbineSteadyState(N_turb,omega_of_g,H_turb0)

global use_simple_gateflow_model use_constant_turbine_efficiency...
    z_forebay z_turbine...
    d_runner...
    G_min G_max...
    pilot_min pilot_max...
    H_min H_max...
    Power_max...
    a_g rho...
    omega_m_nom...
    f_p sound_speed L_penstock A_penstock d_penstock...
    gate_flow_coeff...
    k_pilotservo k_feedback k_mainbooster T_mainbooster k_mainservo...
    PID_Kp PID_Ki PID_Kd...
    omega_dead_zone...
    turbine_eta_m G_nq_q_f turbine_const_efficiency...
    part_load_freq n_over_q_required G_required...
    osc_Gs osc_heads osc_tstep T_w T_e T_m runner_ibertia...
    z_t1 z_t2 z_t3 Q_base G_base pilot_base G_n_q_f;

% % % steady state is computed from the turbine initial power,
% rotational speed, and turbine head

assert((H_turb0>0)&&(H_turb0<220),'initial turbine head is wrong');

%% % % % % get Q from q_i,H % % % % % %
Q_f=@(q_i,H)((q_i.*sqrt(H)*d_runner^2)/1000); % m3/s
rads_rpm =@(omega)(60*omega/(2*pi)); % rad/s to rpm
n_i = @(g)(rads_rpm(omega_of_g(g))*d_runner/sqrt(H_turb0));
q_i = @(g)(gate_flow_coeff*g*G_base*1000/(d_runner^2));
if ~use_simple_gateflow_model
    q_i = @(g)(G_n_q_f(g*G_base,n_i(g)));
end
efficiency = @(g)(turbine_const_efficiency);
if ~use_constant_turbine_efficiency
    efficiency = @(g)(turbine_eta_m(q_i(g),n_i(g)));
end
g_guess = 0.5;
power = @(g)(a_g*rho*efficiency(g)*H_turb0*Q_f(q_i(g),H_turb0));
g0=fzero(@(g)(N_turb-power(g)),g_guess);
G0 = g0*G_base;
Q0 = Q_f(q_i(g0),H_turb0);
q0 = Q0/Q_base;

% test = (N_of_g -rho*a_g*efficiency*H_turb0*Q0)/omega_m_nom;
% assert(abs(test)<0.001,'power is wrong, N0=%3.1e',N_of_g);
if (Q0<0 || Q0>358)
    fprintf('WARNGING: Q_0 is wrong, Q0=%.1f\n',Q0);
end
assert((G0>=0)&&(G0<=G_max),'wrong initial gate opening, G0=%.0f',G0);

%% compute tailrace height which corresponds to turbine head and forebay height
% Darcy-Weisbach friction head loss
H_loss_0 = f_p*L_penstock*8*Q0^2/((pi^2)*(d_penstock^5)*a_g);
% H_loss = f_p*L_penstock*8*Q^2/((pi^2)*(d_penstock^5)*a_g);
assert((H_loss_0>=0)&&(H_loss_0<=5),'head loss is wrong');

% tailrace level must be compatible with all other parameters
z_tailrace = z_forebay-H_turb0-H_loss_0;
assert(z_tailrace>z_t1,'tailrace is wrong %.1f',z_tailrace);

end