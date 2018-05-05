function [] = printState(t,state,gen_model,turb_model,load_model,gov_model)
%printState prints state to consolse in a pretty way
fprintf('(');
fprintf('%.2f,',state(1:end-1));
fprintf('%.2f)\n',state(end));
[omega_m,q,g,governer_state,psi,exciter_state] = parseState(state,gov_model.state_size);
Q_base = turb_model.Q_base;
G_base = turb_model.G_base;
S_base = gen_model.S_base;

[E_q,E_rq,E_rd,i_q,i_d] = gen_model.psi_to_E(psi);
[v_d,v_q] = load_model.model(t,i_d,i_q,omega_m,gen_model.omega_m_nom,gen_model.S_base);
[dq,Turbine_power,H_turb,H_loss] = turb_model.model(0,g,q,omega_m);
fprintf('omega_m=%.1f(rad/s) Q=%.0f(m^3/s) G=%.0f(mm) H_t=%.0f(m) H_{loss}=%.1f(m)\n',...
    omega_m,q*Q_base,g*G_base,H_turb,H_loss);
P_active = (v_d*i_d+v_q*i_q)*S_base/10^6;
Q_reactive = (v_q*i_d-v_d*i_q)*S_base/10^6;
fprintf('e_r=%.1f\n',state(11));
fprintf('active power %.0fMW, reactive power %.0fMW\n',P_active,Q_reactive);
end

