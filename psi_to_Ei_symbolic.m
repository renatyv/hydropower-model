clear all;
syms x_d x_q x_0 x_ad x_aq x_r x_D x_Q x_rd x_rq...
    psi_d psi_q psi_r psi_rd psi_rq...
    e_q e_rq e_rd i_q i_d;

eq1 = psi_d == x_d*i_d+e_q+e_rq;
eq2 = psi_q == x_q*i_q-e_rd;
eq3 = psi_r == (x_ad^2)/x_r*i_d+e_q+x_ad/x_r*e_rq;
eq4 = psi_rd == x_ad^2/x_rd*i_d+e_rq+x_ad/x_rd*e_q;
eq5 = psi_rq == x_aq^2/x_rq*i_q-e_rd;

sol = solve([eq1,eq2,eq3,eq4,eq5],[e_q,e_rq,e_rd,i_q,i_d]);