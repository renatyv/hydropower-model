%% simulateModel sets paremeters
% for system and starts modelling
%
global  S_base torque_base cos_phi Q_nom U_nom...
        x_ad x_q x_d x_aq...
		r_s x_s x_dp x_dpp x_qpp...
        T_r T_dpp T_qpp x_r x_sr...
        x_rd x_rq r_rd r_rq T_rd T_rq...
		poles_number...
        omega_er_base...
        omega_m_nom...
        V_base I_base R_base...
        rotor_inertia;
%% Modeling parameters for generator from "Reference on electrical networks"
    rotor_inertia = 102000*10^3; %(kg*m^2) rotor inertia
	poles_number = 21; % Number of pole-pairs
% 	S_base = 640*10^6; % Nominal power, Watt
	cos_phi = 0.9; % nominal angle between voltage and current
	Q_nom = 315*10^6; % nominal reacitv power, Mvar. Q = V_rms*I_rms*sin_phi
	U_nom = 15.75*1000; % Nominal stator voltage in volts, RMS voltage
    V_base = sqrt(2)*U_nom;
    I_base = S_base/(3/2*V_base); %
    R_base = V_base/I_base;
    omega_er_base = omega_m_nom*poles_number;
    torque_base = S_base/omega_m_nom;
%     v_f = 375; % From reference on generators p. 307
%     R_base % base resistance
% resistancies
    r_s = 0.0034; % stator resistance
    x_s = 0.184; % leakage inductive resistance
    x_d = 1.58; % Xd
    x_q = 0.97;
%     original parameters
    x_dp = 0.295; % Xd'
    x_dpp = 0.43; % Xd''
%     x_dp = 0.43; % Xd'
%     x_dpp = 0.295; % Xd''
    x_qpp = 0.31;
    T_r = 8.21;
    T_dpp=0.143;
    T_qpp=0.243;
    E_r_nom = 530;
    x_ad = x_d-x_s;
    x_aq = x_q-x_s;
    x_r = (x_ad^2)/(x_d-x_dp);
    x_sr = x_r-x_ad;
    x_rd = x_ad+1/(1/(x_dpp-x_s)-1/x_ad-1/x_sr);
    x_rq = x_aq+1/(1/(x_qpp-x_s)-1/x_aq);
    r_rd = (x_rd*x_d-x_ad^2)*x_rd/(omega_er_base*x_d*x_dp*T_dpp);
    r_rq = (x_rq*x_q-x_aq^2)/(omega_er_base*x_q*T_qpp);
    T_rd = x_rd/(omega_er_base*r_rd);
    T_rq = x_rq/(omega_er_base*r_rq);