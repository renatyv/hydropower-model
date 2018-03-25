function [ dq,Turbine_power,H_turb,H_loss] = turbineModel(t,g,q,omega_m)
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
    N_turb_const z_tailrace_const;


if constant_turbine_torque
    dq = 0;
    Turbine_power = N_turb_const;
    H_turb = z_forebay-z_tailrace_const;
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

    %% % % Compute H_turb %%%%%
    % turbine speed rpm 
    n=60*omega_m/(2*pi);
    H_turb = (Q/(gate_flow_coeff*G))^2;
    q_i = 1000*Q/(d_runner^2*sqrt(H_turb));
    if ~use_simple_gateflow_model
        n_over_q = n*(d_runner^3)/(1000*Q);
        q_i = G_nq_q_f(G,n_over_q);
        H_turb = (1000*Q/((d_runner^2)*q_i))^2;
    end
    
    %% compute loss and static head
    % fprintf('z_tailrace=%.f\n',z_tailrace);
    H_st = z_forebay-z_tailrace_const+H_osc;
    H_loss = f_p*L_penstock*8*Q^2/((pi^2)*(d_penstock^5)*a_g);
    
    dQ = (A_penstock*a_g/L_penstock)*(H_st-H_loss-H_turb);
    dq = dQ/Q_base;

    %% % %  compute turbine torque
    n_i = n*d_runner/sqrt(H_turb);
    turbine_efficiency = turbine_const_efficiency;
    if ~use_constant_turbine_efficiency
        turbine_efficiency = turbine_eta_m(q_i,n_i);
    end
    Turbine_power = rho*a_g*Q*H_turb*turbine_efficiency;
end
end