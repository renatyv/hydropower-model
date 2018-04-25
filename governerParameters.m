global Pilot_base...
    Pilot_max Pilot_min  omega_dead_zone...
    T_m T_w T_mainservo T_pilotservo...
    omega_gov_ref omega_m_nom;
%% dead zone 
omega_dead_zone = 5*omega_m_nom*10^-5;
omega_gov_ref = omega_m_nom+0.35;

%% recommended PID parameters
Kp = 0.8*T_m/T_w; % 1.68
Ki = 0.24*T_m/T_w^2; % 0.57
Kd = 0.27*T_m; % 0.5
fprintf('recommended PID Kp=%.1f, Ki=%.1f, Kd=%.1f\n',Kp,Ki,Kd);

%% pilot and main servo parameters
Pilot_max = 120;
Pilot_min = 0;
Pilot_base = Pilot_max;

T_mainservo = 0.76;
T_pilotservo = T_mainservo*1.3;