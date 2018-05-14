function [dServo_state] = servoModel(input,servo_state,servo_min,servo_max,T_d,enable_saturation)
%SERVOMODEL Models servo with optional saturation
%   transfer function F(s)=1/(T_d*s +1)
% steady state: input = servo_state
dServo_state = (-servo_state + input)/T_d;
if enable_saturation && ((servo_state<=servo_min && dServo_state<=0) || (servo_state>=servo_max && dServo_state>=0))
    dServo_state=0;
end
end

