function [near_state] = generate_state_near(state)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
global gate_flow_coeff Q_base G_base;
q = state(2);
g = state(3);
Q=q*Q_base;
G=g*G_base;
H_turb_steady = (Q/(gate_flow_coeff*G))^2;
dState = (rand(size(state))-0.5)*0.5;
dH = dState(2)*200;
Q_new = (q+dState(3))*Q_base;
G_new = Q_new/(sqrt((H_turb_steady+dH))*gate_flow_coeff);
near_state = state+dState;
near_state(3) = G_new/G_base;
end

