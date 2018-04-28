function [e_q,e_rq,e_rd,i_q,i_d] = psi_to_E(psi)
%I_TO_PSI Solve flux-linkage equations
%   Detailed explanation goes here
global x_d x_q x_ad x_aq x_r x_rd x_rq;
[psi_d,psi_q,psi_r,psi_rd,psi_rq] = parsePsi(psi);
e_q = -(psi_d*x_ad^3 - psi_r*x_ad^2*x_r - psi_d*x_ad^2*x_rd + psi_rd*x_ad^2*x_rd - psi_rd*x_ad*x_d*x_rd + psi_r*x_d*x_r*x_rd)/...
    (x_ad^2*x_d + x_ad^2*x_r + x_ad^2*x_rd - 2*x_ad^3 - x_d*x_r*x_rd);
e_rq =-(psi_d*x_ad^3 - psi_d*x_ad^2*x_r + psi_r*x_ad^2*x_r - psi_rd*x_ad^2*x_rd - psi_r*x_ad*x_d*x_r + psi_rd*x_d*x_r*x_rd)/...
    (x_ad^2*x_d + x_ad^2*x_r + x_ad^2*x_rd - 2*x_ad^3 - x_d*x_r*x_rd);
e_rd = (psi_q*x_aq^2 - psi_rq*x_q*x_rq)/(- x_aq^2 + x_q*x_rq);
i_q = (x_rq*(psi_q - psi_rq))/(- x_aq^2 + x_q*x_rq);
i_d = (psi_d*x_ad^2 - psi_r*x_ad*x_r - psi_rd*x_ad*x_rd - psi_d*x_r*x_rd + psi_r*x_r*x_rd + psi_rd*x_r*x_rd)/(x_ad^2*x_d + x_ad^2*x_r + x_ad^2*x_rd - 2*x_ad^3 - x_d*x_r*x_rd);
end