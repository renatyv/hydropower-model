function [y] = sat(x,x_min,x_max)
%SAT Summary of this function goes here
%   Detailed explanation goes here
y = max(x_min,min(x_max,x));
end

