function [g0, q0]=turbineSteadyState(N_turb,omega_of_g)
%turbineSteadyState computes turbine steady state from the power
%   omega_of_g --- function that matches static rotor frequency to the gate
%   opening

global Q_base S_base;
function dq = turb_model(x)
    q=x(1);
    g=x(2);
    Q=q*Q_base;
    [ dqq,Turbine_power,~,~] = turbineModel(0,g,q,omega_of_g(g));
    dq = dqq.^2+(N_turb/S_base-Turbine_power/S_base)^2;
end
x=fminsearch(@turb_model,[0.1,0.1]);
q0=x(1);
g0=x(2);
end