function [] = printState(state,gen_model,turb_model)
%printState prints state to consolse in a pretty way
global omega_m_nom G_base Q_base S_base; 
fprintf('(');
fprintf('%.2f,',state(1:end-1));
fprintf('%.2f)\n',state(end));
[omega_m,q,g,governer_state,psi,exciter_state] = parseState(state);
% omega_m = state(1);
% q=state(2);
% g = state(3);
PID_i=state(4);
ps = state(5);
% psi_d = state(6);
% psi_q = state(7);
% psi_r = state(8);
% psi_rd = state(9);
% psi_rq = state(10);

[E_q,E_rq,E_rd,i_q,i_d] = gen_model.psi_to_E(psi);
[v_d,v_q] = loadModel(0,i_d,i_q,omega_m,gen_model.S_base,gen_model.omega_m_nom);
[dq,Turbine_power,H_turb,H_loss] = turb_model.model(0,g,q,omega_m);
fprintf('omega_m=%.1f(rad/s) Q=%.0f(m^3/s) G=%.0f(mm) H_t=%.0f(m) H_{loss}=%.1f(m) PID_i=%.2f ps=%.1f\n',...
    omega_m,q*Q_base,g*G_base,H_turb,H_loss,...
    PID_i,ps);
P_active = (v_d*i_d+v_q*i_q)*S_base/10^6;
Q_reactive = (v_q*i_d-v_d*i_q)*S_base/10^6;
fprintf('e_r=%.1f\n',state(11));
fprintf('active power %.0fMW, reactive power %.0fMW\n',P_active,Q_reactive);
end

