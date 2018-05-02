classdef GenModel

%     global  S_base torque_base cos_phi Q_nom U_nom...
%             x_ad x_q x_d x_aq...
%             r_s x_s x_dp x_dpp x_qpp...
%             T_r T_dpp T_qpp x_r x_sr...
%             x_rd x_rq r_rd r_rq T_rd T_rq...
%             poles_number...
%             omega_er_base...
%             omega_m_nom...
%             V_base I_base R_base...
%             rotor_inertia;
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
end