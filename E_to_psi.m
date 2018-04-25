function [psi_d,psi_q,psi_r,psi_rd,psi_rq] = E_to_psi(e_q,e_rq,e_rd,i_q,i_d)
%I_TO_PSI flux-linkage equations
%   Detailed explanation goes here
global x_d x_q x_ad x_aq x_r x_rd x_rq;
psi_d = x_d.*i_d+e_q+e_rq;
psi_q = x_q.*i_q-e_rd;
psi_r = (x_ad.^2)/x_r.*i_d+e_q+x_ad./x_r.*e_rq;
psi_rd = (x_ad.^2)/x_rd.*i_d+e_rq+x_ad/x_rd.*e_q;
psi_rq = (x_aq.^2)./x_rq.*i_q-e_rd;
end