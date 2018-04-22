function [near_state] = generate_state_near(state,epsilon)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
global gate_flow_coeff Q_base G_base Pilot_base Pilot_max Pilot_min G_min G_max S_base;
pilot_max = Pilot_max/Pilot_base;
pilot_min = Pilot_min/Pilot_base;
g_max = G_max/G_base;
g_min = G_min/G_base;

% H_turb_steady = (Q/(gate_flow_coeff*G))^2;
H= -0.5;

P_active = 0;

while H<100 || H>300 || P_active<0 || P_active>700
    dState = (rand(size(state))-0.5)*epsilon;
    near_state = state+dState;
%%     saturate the servos
    near_state(5) = sat(near_state(5),pilot_min,pilot_max);
    near_state(3) = sat(near_state(3),g_min,g_max);
    G = near_state(3)*G_base;
    Q = near_state(2)*Q_base;
    H=(Q/(gate_flow_coeff*G))^2;
    
    omega_m = near_state(1);
    psi_d = near_state(6);
    psi_q = near_state(7);
    psi_r = near_state(8);
    psi_rd = near_state(9);
    psi_rq = near_state(10);
    [E_q,E_rq,E_rd,i_q,i_d] = psi_to_E(psi_d,psi_q,psi_r,psi_rd,psi_rq);
    [v_d,v_q] = loadModel(0,i_d,i_q,omega_m);
    P_active = (v_d*i_d+v_q*i_q)*S_base/10^6;
    Q_reactive = (v_q*i_d-v_d*i_q)*S_base/10^6;
end

end

