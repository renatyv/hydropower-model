function [omega_m,q,g,governer_state,psi,exciter_state] = parseState(state,gov_size)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
omega_m = state(1);
q = state(2);
g = state(3);
state_size = 3;
governer_state = state(state_size+1:state_size+gov_size);
state_size = state_size+gov_size;
psi = state(state_size+1:state_size+5);
state_size = state_size+5;
exciter_state  = state(state_size+1);
end