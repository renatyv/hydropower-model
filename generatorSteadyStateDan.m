function [psi_1,e_r_1, psi_2,e_r_2] =...
    generatorSteadyStateDan(i0, phi_1,omega_er0)
%generatorSteadyStateDan computes steady state of the generator
%   i0 --- current amplitude,
%   phi_1 --- power coefficient, 
%   omega_er0 --- rotor frequency
global  x_0 x_D x_Q x_f x_ad x_q x_d x_aq omega_er_base...
		r_s r_0 r_f I_ampl_base x_rq x_r x_rd S_base V_base I_base;
%% compute steady state corresponding to initial load
    v_df = @(theta)(cos(theta));
    v_qf = @(theta)(-sin(theta));
    i_df = @(theta)(i0*cos(theta+phi_1));
    i_qf = @(theta)(-i0*sin(theta+phi_1));
    omega_pu0 = omega_er0/omega_er_base;
% % function is 2pi-periodic, there are two zeros
    theta_f = @(theta_er)(omega_pu0*x_q*i_qf(theta_er)+i_df(theta_er)*r_s+v_df(theta_er));
%     options = optimset('Display','iter'); % show iterations
    theta_er1 = fzero(theta_f,[-pi,0]);
    theta_er2 = fzero(theta_f,[0,pi]);
    function [psi,e_r] = compute_state(theta)
        v_d = v_df(theta);
        v_q = v_qf(theta);
        i_d = i_df(theta);
        i_q = i_qf(theta);
        e_rd = 0;
        e_rq = 0;
        psi_rq = x_aq^2/x_rq*i_q;
        psi_q = x_q*i_q;
        psi_d = (r_s*i_q+v_q)/omega_pu0;
        e_q = psi_d-x_d*i_d;
        e_r = e_q;
        psi_r = (x_ad^2)/x_r*i_d+e_q;
        psi_rd = (x_ad^2)/x_rd*i_d+x_ad/x_rd*e_q;
%         t_shaft = -(psi_d*i_q-psi_q*i_d);
        psi = constructPsi(psi_d,psi_q,psi_r,psi_rd,psi_rq);
        [e_q1,e_rq1,e_rd1,i_q1,i_d1] = psi_to_E(psi);
        test_eps = 10^-6;
        assert(abs(e_q1-e_q)<test_eps,'wrong E_q');
        assert(abs(e_rq1-e_rq)<test_eps,'wrong E_rq');
        assert(abs(e_rd1-e_rd)<test_eps,'wrong E_rd');
        assert(abs(i_q1-i_q)<test_eps,'wrong i_q');
        assert(abs(i_d1-i_d)<test_eps,'wrong i_d');
        
    end
    [psi_1,e_r_1] = compute_state(theta_er1);
    [psi_2,e_r_2] = compute_state(theta_er2);
end