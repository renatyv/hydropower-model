function [near_state] = generate_state_near(gov_model,gen_model,turb_model,state,epsilon)
%generate_state_near generates valid steady state near the state within radius epsilon
% pilot_max = gov_model.Pilot_max/gov_model.Pilot_base;
% pilot_min = gov_model.Pilot_min/gov_model.Pilot_base;
g_max = turb_model.G_max/turb_model.G_base;
g_min = turb_model.G_min/turb_model.G_base;

% H_turb_steady = (Q/(gate_flow_coeff*G))^2;
H= -0.5;

P_active = 0;

while H<100 || H>300 || P_active<0 || P_active>700*10^6
    dState = (rand(size(state))-0.5)*epsilon;
    near_state = state+dState;
%%     saturate the servos
    near_state(5) = sat(near_state(5),gov_model.Pilot_min,gov_model.Pilot_max);
    near_state(3) = sat(near_state(3),g_min,g_max);
    G = near_state(3)*turb_model.G_base;
    Q = near_state(2)*turb_model.Q_base;
    H=(Q/(turb_model.gate_flow_coeff*G))^2;
    
    omega_m = near_state(1);
    psi_d = near_state(6);
    psi_q = near_state(7);
    psi_r = near_state(8);
    psi_rd = near_state(9);
    psi_rq = near_state(10);
    psi = constructPsi(psi_d,psi_q,psi_r,psi_rd,psi_rq);
    [E_q,E_rq,E_rd,i_q,i_d] = gen_model.psi_to_E(psi);
    [v_d,v_q] = loadModel(0,i_d,i_q,omega_m,gen_model.S_base,gen_model.omega_m_nom);
    P_active = (v_d*i_d+v_q*i_q)*gen_model.S_base;
    Q_reactive = (v_q*i_d-v_d*i_q)*gen_model.S_base;
end

end

