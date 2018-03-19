function [value,isterminal,direction] = stop_integration_event( t,state )
%STOP_INTEGRATION_EVENT Summary of this function goes here
%   Detailed explanation goes here
global G_max pilot_max Q_max gate_flow_coeff H_max...
        G_base Q_base omega_m_nom;
omega_m = state(1);
G=state(2)*G_base;
Q=state(3)*Q_base;
PID_i=state(4);
pilot_servo = state(5); 
% psi_d = state(6);
% psi_q = state(7);
% psi_0 = state(8);
% psi_f = state(9);
% psi_D = state(10);
% psi_Q = state(11);
% theta_er = state(12);
% e_1 = state(13);
% e_2 = state(14);

% check that all of them are non-negative
H_turb = (Q/(gate_flow_coeff*G))^2;
% value=min([G,(G_max-G),Q,(Q_max-Q),H_turb,(H_max-H_turb),omega_m,(omega_m_nom-omega_m)]);% value(i) is the value of the ith event function.
value=min([G,(2*G_max-G),Q,(2*Q_max-Q),H_turb,(2*H_max-H_turb),omega_m,(2*omega_m_nom-omega_m)]);% value(i) is the value of the ith event function.

if value<0 
%     print_state(state);
    fprintf('hydraulic system is out of bounds, G=%.1f Q=%.1f H=%.1f omega_m=%.1f\n',G,Q,H_turb,omega_m);
end
isterminal = 1; % isterminal(i) = 1 if the integration is to terminate at a zero of this event function. Otherwise, it is 0.
direction = 0; % direction(i) = 0 if all zeros are to be located (the default)

end

