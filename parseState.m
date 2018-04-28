function [omega_m,q,g,governer_state,psi,exciter_state] = parseState(state)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
omega_m = state(1);
q = state(2);
g = state(3);
governer_state = state(4:5);
psi = state(6:10);
exciter_state  = state(11);
end