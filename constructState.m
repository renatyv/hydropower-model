function [state] = constructState(omega_m,q,g,governer_state,psi,exciter_state)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
state=[omega_m/GenModel.omega_m_nom;q;g;governer_state;psi;exciter_state];
end

