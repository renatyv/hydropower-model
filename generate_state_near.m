function [near_state] = generate_state_near(state)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
global gate_flow_coeff Q_base G_base Pilot_base Pilot_max Pilot_min G_min G_max;
pilot_max = Pilot_max/Pilot_base;
pilot_min = Pilot_min/Pilot_base;
g_max = G_max/G_base;
g_min = G_min/G_base;

% H_turb_steady = (Q/(gate_flow_coeff*G))^2;
H= -0.5;
while H<100 || H>300
    dState = (rand(size(state))-0.5)*0.5;
    near_state = state+dState;
%%     saturate the servos
    near_state(5) = sat(near_state(5),pilot_min,pilot_max);
    near_state(3) = sat(near_state(3),g_min,g_max);
    G = near_state(3)*G_base;
    Q = near_state(2)*Q_base;
    H=(Q/(gate_flow_coeff*G))^2;
end

end

