function [dServo_state] = saturatedIntegrator(input,servo_state,servo_min,servo_max,k,enable_saturation)
%SERVOMODEL Models servo with optional saturation
%   transfer function F(s)=k/s
dServo_state = k*input;
if enable_saturation && ((servo_state<=servo_min && dServo_state<=0) || (servo_state>=servo_max && dServo_state>=0))
    dServo_state=0;
end
end

