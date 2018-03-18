function [dServo_state] = servoModel(input,servo_state,servo_min,servo_max,T_d)
%SERVOMODEL Summary of this function goes here
%   Detailed explanation goes here
dServo_state = (-servo_state + input)/T_d;
if (servo_state<=servo_min && dServo_state<=0) || (servo_state>=servo_max && dServo_state>=0)
    dServo_state=0;
end
end

