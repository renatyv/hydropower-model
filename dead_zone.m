function y = dead_zone(x,a,b)
%DEAD_ZONE Summary of this function goes here
%   Detailed explanation goes here
if (x<=a) || (x>=b)
    y=x;
else
    y=0.0;
end
end

