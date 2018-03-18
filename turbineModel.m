function [ dq,Turbine_torque,H_turb] = turbineModel(t,g,q,omega_m,z_tailrace)
%HYDRAULIC_MODEL Summary of this function goes here
%   Detailed explanation goes here
global use_simple_gateflow_model use_constant_turbine_efficiency constant_turbine_torque...
    z_forebay...
    d_runner...
    G_min G_max pilot_min pilot_max...
    a_g rho...
    f_p L_penstock A_penstock d_penstock...
    gate_flow_coeff...
    turbine_const_efficiency turbine_eta_m G_nq_q_f...
    Q_base H_base S_base torque_base G_base...
    osc_Gs osc_ampl...
    simulate_vortex_rope_oscillations...
    omega_m_nom N_turb_const omega_m_const;


if constant_turbine_torque
    dq = 0;
    Turbine_torque = N_turb_const/omega_m_const;
    H_turb = z_forebay-z_tailrace;
else 
    Q=q*Q_base;
    G=g*G_base;
    G=max(G,G_min);
    G=min(G,G_max);

    %% simulate head oscillations
    H_osc = 0;
    if simulate_vortex_rope_oscillations
%         osc_Hz = 0.7; %Hz
%         omega_osc = osc_Hz*2*pi;
        omega_osc = 0.4*omega_m;
        H_osc = sin(omega_osc*t)*spline(osc_Gs,osc_ampl,G);
    end

    %% compute loss and static head
    % fprintf('z_tailrace=%.f\n',z_tailrace);
    H_st = z_forebay-z_tailrace+H_osc;
    H_loss = f_p*L_penstock*8*Q^2/((pi^2)*(d_penstock^5)*a_g);

    %% % % Compute H_turb %%%%%
    % turbine speed rpm 
    n=60*omega_m/(2*pi);
    q_i = 0;
    H_turb = (Q/(gate_flow_coeff*G))^2;
    if ~use_simple_gateflow_model
        n_over_q = n*(d_runner^3)/(1000*Q);
        q_i = G_nq_q_f(G,n_over_q);
        H_turb = (1000*Q/((d_runner^2)*q_i))^2;
    end
    dQ = (A_penstock*a_g/L_penstock)*(H_st-H_loss-H_turb);
    dq = dQ/Q_base;

    %% % %  compute turbine torque
    n_i = n*d_runner/sqrt(H_turb);
    turbine_efficiency = turbine_const_efficiency;
    if ~use_constant_turbine_efficiency
        turbine_efficiency = turbine_eta_m(q_i,n_i);
    end
    Turbine_torque = rho*a_g*Q*H_turb*turbine_efficiency/omega_m;
end
end