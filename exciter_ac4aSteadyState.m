function [exciter_steady_state] = exciter_ac4aSteadyState(e_r)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
global K_A;
exciter_steady_state(1)=e_r/K_A;
exciter_steady_state(2)=e_r;
end