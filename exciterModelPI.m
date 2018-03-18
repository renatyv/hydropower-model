function [e_r,dexciter_state] = exciterModelPI(v_q,v_d,exciter_state)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
global exciter_PID_Ki exciter_PID_Kp v_rmin v_rmax constant_exciter e_r_const;
if constant_exciter
    e_r = e_r_const;
    dexciter_state = 0;
else
    v_error = 1-sqrt(v_q^2+v_d^2);
    dexciter_state = exciter_PID_Ki*v_error;
    e_r = exciter_PID_Kp*v_error + exciter_state;
end
end