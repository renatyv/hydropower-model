function [] = printState(state)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
global omega_m_nom G_base Q_base S_base; 
omega_m = state(1);
q=state(2);
g = state(3);
PID_i=state(4);
ps = state(5);
psi_d = state(6);
psi_q = state(7);
psi_r = state(8);
psi_rd = state(9);
psi_rq = state(10);

[E_q,E_rq,E_rd,i_q,i_d] = psi_to_E(psi_d,psi_q,psi_r,psi_rd,psi_rq);
[v_d,v_q] = loadModel(0,i_d,i_q,omega_m);
[dq,Turbine_power,H_turb,H_loss] = turbineModel(0,g,q,omega_m);
fprintf('omega_m=%.1f(rad/s) Q=%.0f(m^3/s) G=%.0f(mm) H_t=%.0f(m) H_{loss}=%.1f(m) PID_i=%.2f ps=%.1f\n',...
    omega_m,q*Q_base,g*G_base,H_turb,H_loss,...
    PID_i,ps);
% fprintf('psi_d=%.1f, psi_q=%.1f, psi_r=%.1f, psi_rd=%.1f, psi_rq=%.1f\n',...
%     psi_d,psi_q,psi_r,psi_rd,psi_rq);
% fprintf('v_d0=%.1f v_q0=%.1f i_d0=%.1f i_q0=%.1f\n',...
%     v_d,v_q,i_d,i_q);
P_active = (v_d*i_d+v_q*i_q)*S_base/10^6;
Q_reactive = (v_q*i_d-v_d*i_q)*S_base/10^6;
fprintf('active=%.0fMW reactive=%.0fMW\n',P_active,Q_reactive);
fprintf('e_1=%.1f\n',state(11));
% dState = full_model(0,state);
% fprintf('domega_m=%.1e(rad/s) dg=%.0e(mm) dq=%.0e(m^3/s) dPID_i=%.1e dps=%.1e\n',...
%     dState(1),dState(2),dState(3),dState(4),dState(5));
% fprintf('dpsi_d=%.1e, dpsi_q=%.1e, dpsi_r=%.1e, dpsi_rd=%.1e, dpsi_rq=%.1e\n',...
%     dState(6),dState(7),dState(8),dState(9),dState(10));
% fprintf('de_1=%.1e\n',dState(11));
end

