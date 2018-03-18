function [dx] = estimateDerivative(x,t)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
dx=zeros(size(x));
dx(1) = (x(2)-x(1))/(t(2)-t(1));
for i=2:length(t)-1
    dx1 = (x(i+1)-x(i))/(t(i+1)-t(i));
    dx2 = (x(i)-x(i-1))/(t(i)-t(i-1));
    dx(i)=(dx1+dx2)/2;
end
dx(end) = (x(end)-x(end-1))/(t(end)-t(end-1));
end

