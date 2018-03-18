function [PID_i0,PID_d0, pilot_servo_0]=...
    governerSteadyState(g0)

global PID_Ki k_feedback;

% % % % % servos initial state
pilot_servo_0 = g0;

% % PID initial conditions correspond to equilibrium
% integrator
PID_i0 = g0/PID_Ki;
PID_d0 = 0;
end