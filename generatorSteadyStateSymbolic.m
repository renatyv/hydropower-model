clear all;

syms theta omega t theta_0 phi_1 I_ampl V_ampl V_a V_b V_c V_d V_q V_0 U_nom P_active1 Q_reactive1 P_active2 Q_reactive2 V_dq0 I_dq0;
assume(theta,'real');
assume(t,'real');
assume(omega,'positive');
% assume(V_ampl,'real');
assume(V_ampl,'positive');
% assume(I_ampl,'real');
assume(I_ampl,'positive');
% assume(U_nom,'real');
assume(U_nom,'positive');

theta = omega*t+theta_0;
% V_ampl = U_nom/sqrt(2);

P = 2/3*[cos(-theta),cos(-theta+2/3*pi),cos(-theta-2/3*pi);...
    sin(-theta),sin(-theta+2/3*pi),sin(-theta-2/3*pi);...
    1/2,1/2,1/2];

P1 = 3/2*P';
P2 = [cos(-theta),sin(-theta),1;...
    cos(-theta + 2/3*pi),sin(-theta + 2/3*pi),1;...
    cos(-theta - 2/3*pi),sin(-theta - 2/3*pi),1];
simplify(P*P2)

V_a = V_ampl*cos(omega*t);
V_b = V_ampl*cos(omega*t-2*pi/3);
V_c = V_ampl*cos(omega*t+2*pi/3);
I_a = I_ampl*cos(omega*t-phi_1);
I_b = I_ampl*cos(omega*t-phi_1-2*pi/3);
I_c = I_ampl*cos(omega*t-phi_1+2*pi/3);
% V_a = V_ampl*cos(theta);
% V_b = V_ampl*cos(theta-2*pi/3);
% V_c = V_ampl*cos(theta+2*pi/3);
% I_a = I_ampl*cos(theta-phi_1);
% I_b = I_ampl*cos(theta-phi_1-2*pi/3);
% I_c = I_ampl*cos(theta-phi_1+2*pi/3);

V_dq0 = simplify(P*[V_a;V_b;V_c]);
V_d = V_dq0(1)
V_q = V_dq0(2)
V_0 = V_dq0(3);

I_dq0 = simplify(P*[I_a;I_b;I_c]);
I_d = I_dq0(1)
I_q = I_dq0(2)
I_0 = I_dq0(3);

P_active1 = simplify(V_a*I_a+V_b*I_b+V_c*I_c)
P_active2 = 3/2*simplify(V_d*I_d+V_q*I_q)
Q_reactive1 = 3/2*simplify(V_q*I_d-V_d*I_q)

v_error = simplify(sqrt(V_d^2+V_q^2))

% simplify(P*diff(P2,t))
