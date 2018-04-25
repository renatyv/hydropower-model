% clear all;
% global use_simple_gateflow_model use_constant_turbine_efficiency constant_turbine_torque...
%     z_forebay...
%     d_runner...
%     G_min G_max pilot_min pilot_max...
%     a_g rho...
%     f_p L_penstock A_penstock d_penstock...
%     gate_flow_coeff...
%     turbine_const_efficiency turbine_eta_m G_nq_q_f...
%     Q_base H_base S_base torque_base G_base...
%     osc_Gs osc_ampl...
%     simulate_vortex_rope_oscillations...
%     N_turb_const z_tailrace_const;
% 
% rpm_nom = 142.8; % revolutions per minute nomial frequency
% omega_m_nom = rpm_nom/60*2*pi; %(rad/s), mechanical frequency of the rotor
% z_forebay = 539;
% z_tailrace_const = 331;
% turbine_characteristics;
% constant_turbine_torque = false;
% use_simple_gateflow_model = false;
% use_constant_turbine_efficiency = false;
% 
% simulate_vortex_rope_oscillations = false;

q=0.7;
g=0.7;
omegas = omega_m_nom:-0.1:1;
Torques = zeros(size(omegas));
for k=1:length(omegas)
    omega=omegas(k);
    [ dq,Turbine_power,H_turb,H_loss] = turbineModel(0,g,q,omega);
    disp([ dq,Turbine_power,H_turb,H_loss]);
    Torques(k) = Turbine_power/omega/10^6;
end

plot(omegas,Torques);