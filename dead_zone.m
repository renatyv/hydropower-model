function y = dead_zone(x,a,b)
%DEAD_ZONE governer dead zone
%   zero within (a,b), linear outside of (a,b)
if (x<=a) || (x>=b)
    y=x;
else
    y=0.0;
end
end

