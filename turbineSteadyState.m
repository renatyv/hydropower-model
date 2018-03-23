function [g0, q0]=turbineSteadyState2(N_turb,omega_of_g,z_tailrace)

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
    z_t1 z_t2 z_t3 Q_base G_base pilot_base G_n_q_f S_base;

% % % steady state is computed from the turbine initial power,
% rotational speed, and turbine head

function dq = turb_model(x)
    q=x(1);
    g=x(2);
    Q=q*Q_base;
    [ dqq,Turbine_power,~] = turbineModel(0,g,q,omega_of_g(g),z_tailrace);
    dq = dqq.^2+(N_turb/S_base-Turbine_power/S_base)^2;
end
x=fminsearch(@turb_model,[0.5,0.5]);
q0=x(1);
g0=x(2);
end