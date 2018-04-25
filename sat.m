function [y] = sat(x,x_min,x_max)
%SAT saturation
%   Detailed explanation goes here
y = max(x_min,min(x_max,x));
end

