function [e_r,dexciter_state] = exciterModelPI(v_q,v_d,exciter_state,enable_saturation,e_r_const)
%exciterModelPI PI model of exciter governer with saturation
global exciter_PID_Ki exciter_PID_Kp constant_exciter;
v_emin = -5;
v_emax = 5;
v_rmin = -1;
v_rmax = 1;
if constant_exciter
    e_r = e_r_const;
    dexciter_state = 0;
else
    v_error = 1-sqrt(v_q^2+v_d^2);
    if enable_saturation
        v_error = sat(v_error,v_rmin,v_rmax);
    end
    
    dexciter_state = exciter_PID_Ki*v_error;
    if enable_saturation && (dexciter_state>0 && exciter_state>=v_emax) || (dexciter_state<0 && exciter_state<=v_emin)
        dexciter_state = 0;
    end
    e_r = exciter_PID_Kp*v_error + exciter_state;
end
end