function [] = printState(state)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
global omega_m_nom G_base Q_base; 
omega_m = state(1);
psi_d0 = state(6);
psi_q0 = state(7);
psi_r0 = state(8);
psi_rd0 = state(9);
psi_rq0 = state(10);

[E_q0,E_rq0,E_rd0,i_q0,i_d0] = psi_to_E(psi_d0,psi_q0,psi_r0,psi_rd0,psi_rq0);
[v_d0,v_q0] = loadModel(0,i_d0,i_q0,omega_m);
fprintf('omega_m=%.1f(rad/s) G=%.0f(mm) Q=%.0f(m^3/s) PID_i=%.1f ps=%.1f\n',...
    state(1),state(3)*G_base,state(2)*Q_base,...
    state(4),state(5));
fprintf('psi_d=%.1f, psi_q=%.1f, psi_r=%.1f, psi_rd=%.1f, psi_rq=%.1f\n',...
    state(6),state(7),state(8),state(9),state(10));
fprintf('v_d0=%.1f v_q0=%.1f i_d0=%.1f i_q0=%.1f\n',...
    v_d0,v_q0,i_d0,i_q0);
fprintf('e_1=%.1f\n',state(11));
% dState = full_model(0,state);
% fprintf('domega_m=%.1e(rad/s) dg=%.0e(mm) dq=%.0e(m^3/s) dPID_i=%.1e dps=%.1e\n',...
%     dState(1),dState(2),dState(3),dState(4),dState(5));
% fprintf('dpsi_d=%.1e, dpsi_q=%.1e, dpsi_r=%.1e, dpsi_rd=%.1e, dpsi_rq=%.1e\n',...
%     dState(6),dState(7),dState(8),dState(9),dState(10));
% fprintf('de_1=%.1e\n',dState(11));
end

