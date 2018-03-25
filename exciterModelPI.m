function [e_r,dexciter_state] = exciterModelPI(v_q,v_d,exciter_state)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
global exciter_PID_Ki exciter_PID_Kp constant_exciter e_r_const;
v_emin = -10;
v_emax = 10;
v_rmin = -4.53;
v_rmax = 5.64;
if constant_exciter
    e_r = e_r_const;
    dexciter_state = 0;
else
    v_error = sat(1-sqrt(v_q^2+v_d^2),v_rmin,v_rmax);
    dexciter_state = exciter_PID_Ki*v_error;
    if (dexciter_state>0 && exciter_state>=v_emax) || (dexciter_state<0 && exciter_state<=v_emin)
        dexciter_state = 0;
    end
    e_r = exciter_PID_Kp*v_error + exciter_state;
end
end