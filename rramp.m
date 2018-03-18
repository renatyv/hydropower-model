function [v] = rramp(v_start,v_end,t_start,t_end,t)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
v=zeros(size(t));
for k=1:length(t)
    if t(k)<t_start
        v(k) = v_start;
    else
        if t(k)>t_end
            v(k)=v_end;
        else
            alpha = (t(k)-t_start)/(t_end-t_start);
            v(k) = v_start+alpha*(v_end-v_start);
        end
    end
end
end

