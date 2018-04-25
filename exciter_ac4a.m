function [e_r,dexciter_state] = exciter_ac4a(v_q,v_d,exciter_state)
%UNTITLED IEEE AC4A exciter model
%   Detailed explanation goes here
global e_r_const v_emin v_emax v_rmin v_rmax T_A T_B T_C K_A v_fref K_C constant_exciter;
dexciter_state = zeros(2,1);
if constant_exciter
    e_r = e_r_const;
    dexciter_state = zeros(2,1);
else
    v_error = sat(1-sqrt(v_q^2+v_d^2),v_emin,v_emax);
    [leadlag_out,dexciter_state(1)] = leadLagFilter(v_error,exciter_state(1),T_C,T_B);
    [dexciter_state(2)] = servoModel(K_A*leadlag_out,exciter_state(2),v_rmin,v_rmax,T_A);
    e_r = exciter_state(2);
end
end