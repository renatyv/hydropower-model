clear all;
%LOAD_MODEL Summary of this function goes here
%   Detailed explanation goes here
syms p q v_d v_q i_q i_d v_d1 v_q1 i_q1 i_d1 theta_er phi_1 theta I_base V_base;
assume(p,'positive');
assume(q,'real');
assume(v_d,'real');
% assume(v_d,'positive');
assume(v_q,'real');
% assume(v_d^2+v_q^2<2);
% assume(i_d,'real');
% assume(i_q,'real');
assume(in(i_d,'real') & in(i_q,'real') & i_d^2+i_q^2>0);
eq1 = p == v_d*i_d+v_q*i_q;
eq2 = q == v_q*i_d-v_d*i_q;

syms V_d V_q I_d I_q
V_d = V_base*cos(theta);
V_q = -V_base*sin(theta);
I_d = I_base*cos(theta+phi_1);
I_q = -I_base*sin(theta+phi_1);
simplify(V_d*I_d+V_q*I_q)
simplify(V_q*I_d-V_d*I_q)

fprintf('static load, constant MVA a=b=0\n');
sol1 = solve([eq1,eq2],[v_d,v_q]);
simplify(sol1.v_d)
simplify(sol1.v_q)

fprintf('static load, constant impedance a=b=2\n');
eq3 = p*(v_d^2+v_q^2) == v_d*i_d+v_q*i_q;
eq4 = q*(v_d^2+v_q^2) == v_q*i_d-v_d*i_q;
sol2 = solve([eq3,eq4],[v_d,v_q]);
simplify(sol2.v_d)
simplify(sol2.v_q)

fprintf('static load, a=1, b=2\n');
eq5 = p*sqrt(v_d^2+v_q^2) == v_d*i_d+v_q*i_q;
eq6 = q*(v_d^2+v_q^2) == v_q*i_d-v_d*i_q;
sol3 = solve([eq5,eq6],[v_d,v_q]);
simplify(sol3.v_d)
simplify(sol3.v_q)



