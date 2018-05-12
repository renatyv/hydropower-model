function [out,dstate] = leadLagFilter(input,state,T_num,T_denum)
%LEADLAGFILTER Summary of this function goes here
% simulates F(s)=(1+sT_num)/(1+sT_denum)
%   Detailed explanation goes here
%  steady state = input = output
dstate = -state/T_denum+input/T_denum;
out = (1-T_num/T_denum)*state+(T_num/T_denum)*input;
end

