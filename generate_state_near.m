function [near_state] = generate_state_near(state)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
global gate_flow_coeff Q_base G_base;
% H_turb_steady = (Q/(gate_flow_coeff*G))^2;
H= -0.5;
while H<100 || H>300
    dState = (rand(size(state))-0.5)*0.5;
    near_state = state+dState;
    q = near_state(2);
    g = near_state(3);
    Q=q*Q_base;
    G=g*G_base;
    H=(Q/(gate_flow_coeff*G))^2;
end
end

