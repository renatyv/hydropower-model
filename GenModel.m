classdef GenModel
    properties(Constant)
        rotor_inertia = 102000*10^3; %(kg*m^2) rotor inertia
        poles_number = 21; % Number of pole-pairs
    	S_base = 640*10^6; % Nominal power, Watt
        cos_phi = 0.9; % nominal angle between voltage and current
        Q_nom = 315*10^6; % nominal reacitv power, Mvar. Q = V_rms*I_rms*sin_phi
        U_nom = 15.75*1000; % Nominal stator voltage in volts, RMS voltage
        V_base = sqrt(2)*GenModel.U_nom;
        I_base = GenModel.S_base/(3/2*GenModel.V_base); %
        R_base = GenModel.V_base/GenModel.I_base;
        omega_m_nom = 14.954;
        omega_er_base = GenModel.omega_m_nom*GenModel.poles_number;
        torque_base = GenModel.S_base/GenModel.omega_m_nom;
    %     v_f = 375; % From reference on generators p. 307
%     %     R_base % base resistance
%     % resistancies
        r_s = 0.0034; % stator resistance
        x_s = 0.184; % leakage inductive resistance
        x_d = 1.58; % Xd
        x_q = 0.97;
%     %     original parameters
        x_dp = 0.295; % Xd'
        x_dpp = 0.43; % Xd''
    %     x_dp = 0.43; % Xd'
%     %     x_dpp = 0.295; % Xd''
        x_qpp = 0.31;
        T_r = 8.21;
        T_dpp=0.143;
        T_qpp=0.243;
        E_r_nom = 530;
        x_ad = GenModel.x_d-GenModel.x_s;
        x_aq = GenModel.x_q-GenModel.x_s;
        x_r = (GenModel.x_ad^2)/(GenModel.x_d-GenModel.x_dp);
        x_sr = GenModel.x_r-GenModel.x_ad;
        x_rd = GenModel.x_ad+1/(1/(GenModel.x_dpp-GenModel.x_s)-1/GenModel.x_ad-1/GenModel.x_sr);
        x_rq = GenModel.x_aq+1/(1/(GenModel.x_qpp-GenModel.x_s)-1/GenModel.x_aq);
        r_rd = (GenModel.x_rd*GenModel.x_d-GenModel.x_ad^2)*GenModel.x_rd/(GenModel.omega_er_base*GenModel.x_d*GenModel.x_dp*GenModel.T_dpp);
        r_rq = (GenModel.x_rq*GenModel.x_q-GenModel.x_aq^2)/(GenModel.omega_er_base*GenModel.x_q*GenModel.T_qpp);
        T_rd = GenModel.x_rd/(GenModel.omega_er_base*GenModel.r_rd);
        T_rq = GenModel.x_rq/(GenModel.omega_er_base*GenModel.r_rq);
    end
    
    properties
        use_constant_torque = false;
        constant_torque = 0;
    end
    
    methods
        function [e_q,e_rq,e_rd,i_q,i_d] = psi_to_E(g,psi)
            %I_TO_PSI Solve flux-linkage equations
            %   Detailed explanation goes here
            [psi_d,psi_q,psi_r,psi_rd,psi_rq] = parsePsi(psi);
            e_q = -(psi_d.*g.x_ad.^3 - psi_r.*g.x_ad.^2.*g.x_r - psi_d.*g.x_ad.^2.*g.x_rd + psi_rd.*g.x_ad.^2.*g.x_rd - psi_rd.*g.x_ad.*g.x_d.*g.x_rd + psi_r.*g.x_d.*g.x_r.*g.x_rd)/...
                (g.x_ad.^2.*g.x_d + g.x_ad.^2.*g.x_r + g.x_ad.^2.*g.x_rd - 2.*g.x_ad.^3 - g.x_d.*g.x_r.*g.x_rd);
            e_rq =-(psi_d.*g.x_ad.^3 - psi_d.*g.x_ad.^2.*g.x_r + psi_r.*g.x_ad.^2.*g.x_r - psi_rd.*g.x_ad.^2.*g.x_rd - psi_r.*g.x_ad.*g.x_d.*g.x_r + psi_rd.*g.x_d.*g.x_r.*g.x_rd)/...
                (g.x_ad.^2.*g.x_d + g.x_ad.^2.*g.x_r + g.x_ad.^2.*g.x_rd - 2.*g.x_ad.^3 - g.x_d.*g.x_r.*g.x_rd);
            e_rd = (psi_q.*g.x_aq.^2 - psi_rq.*g.x_q.*g.x_rq)/(- g.x_aq.^2 + g.x_q.*g.x_rq);
            i_q = (g.x_rq.*(psi_q - psi_rq))/(- g.x_aq.^2 + g.x_q.*g.x_rq);
            i_d = (psi_d.*g.x_ad.^2 - psi_r.*g.x_ad.*g.x_r - psi_rd.*g.x_ad.*g.x_rd - psi_d.*g.x_r.*g.x_rd + psi_r.*g.x_r.*g.x_rd + psi_rd.*g.x_r.*g.x_rd)/(g.x_ad.^2.*g.x_d + g.x_ad.^2.*g.x_r + g.x_ad.^2.*g.x_rd - 2.*g.x_ad.^3 - g.x_d.*g.x_r.*g.x_rd);
            assert(length([e_q,e_rq,e_rd,i_q,i_d])==5,'output size is wrong');
        end
        
        function [psi_1,e_r_1, psi_2,e_r_2] =...
            steady(g,i0, phi_1,omega_er0)
            %generatorSteadyStateDan computes steady state of the generator
            %   i0 --- current amplitude,
            %   phi_1 --- power coefficient, 
            %   omega_er0 --- rotor frequency
            %% compute steady state corresponding to initial load
            v_df = @(theta)(cos(theta));
            v_qf = @(theta)(-sin(theta));
            i_df = @(theta)(i0*cos(theta+phi_1));
            i_qf = @(theta)(-i0*sin(theta+phi_1));
            omega_pu0 = omega_er0/g.omega_er_base;
            % % function is 2pi-periodic, there are two zeros
            theta_f = @(theta_er)(omega_pu0*g.x_q*i_qf(theta_er)+i_df(theta_er)*g.r_s+v_df(theta_er));
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
                psi_rq = g.x_aq^2/g.x_rq*i_q;
                psi_q = g.x_q*i_q;
                psi_d = (g.r_s*i_q+v_q)/omega_pu0;
                e_q = psi_d-g.x_d*i_d;
                e_r = e_q;
                psi_r = (g.x_ad^2)/g.x_r*i_d+e_q;
                psi_rd = (g.x_ad^2)/g.x_rd*i_d+g.x_ad/g.x_rd*e_q;
        %         t_shaft = -(psi_d*i_q-psi_q*i_d);
                psi = constructPsi(psi_d,psi_q,psi_r,psi_rd,psi_rq);
                [e_q1,e_rq1,e_rd1,i_q1,i_d1] = g.psi_to_E(psi);
                test_eps = 10^-6;
                assert(abs(e_q1-e_q)<test_eps,'wrong E_q');
                assert(abs(e_rq1-e_rq)<test_eps,'wrong E_rq');
                assert(abs(e_rd1-e_rd)<test_eps,'wrong E_rd');
                assert(abs(i_q1-i_q)<test_eps,'wrong i_q');
                assert(abs(i_d1-i_d)<test_eps,'wrong i_d');

            end
            [psi_1,e_r_1] = compute_state(theta_er1);
            [psi_2,e_r_2] = compute_state(theta_er2);
            assert(length(psi_1)==5,'psi_1 is undefined');
            assert(length(psi_2)==5,'psi_2 is undefined');
            assert(length(psi_2)==5,'psi_2 is undefined');
            assert(~isempty(e_r_1),'e_r_1 is undefined');
            assert(~isempty(e_r_2),'e_r_2 is undefined');
        end
        
        function [ dpsi,electric_torque] =...
            model(g,psi,v_d,v_q,e_r,omega_m)
            %generatorModel synchronuous generator equations from Merkuriev
            %   Detailed explanation goes here
            assert(length(psi)==5,'psi is wrong, %d',length(psi));
            assert(~isempty(v_d),'v_d is wrong, %d',v_d);
            assert(~isempty(v_q),'v_q is wrong, %d',v_q);
            assert(~isempty(e_r),'e_r is wrong, %d',e_r);
            assert(~isempty(omega_m),'omega_er is wrong, %d',omega_m);
            omega_er = omega_m*g.poles_number;
        %% Vector of derivatives
            if g.use_constant_torque
                dpsi = zeros(5,1);
                electric_torque = g.constant_torque;
            else
                %% Flux linkage equations
                [psi_d,psi_q,~,~,~] = parsePsi(psi);
                [e_q,e_rq,e_rd,i_q,i_d] = g.psi_to_E(psi);
                omega_er_pu = omega_er/g.omega_er_base;
                dpsi_d = g.omega_er_base.*(-omega_er_pu .* psi_q - i_d .* g.r_s - v_d);
                dpsi_q = g.omega_er_base.*(omega_er_pu .* psi_d - i_q .* g.r_s - v_q);
                dpsi_r = 1/g.T_r*(e_r-e_q);
                dpsi_rd = -1/g.T_rd*e_rq;
                dpsi_rq = 1/g.T_rq*e_rd;
                dpsi = constructPsi(dpsi_d,dpsi_q,dpsi_r,dpsi_rd,dpsi_rq);
                electric_torque = -(psi_d.*i_q-psi_q.*i_d)*g.torque_base;
            end
            assert(length(dpsi)==5,'dpsi is wrong, %d',length(dpsi));
            assert(~isempty(electric_torque),'electric_torque is wrong, %.2f',electric_torque);
        end

    end
end