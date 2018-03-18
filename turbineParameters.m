global z_forebay z_turbine z_t1 z_t2 z_t3...
    d_runner...
    G_min G_max...
    H_min H_max...
    Power_max...
    a_g rho...
    omega_m_nom...
    f_p sound_speed L_penstock A_penstock d_penstock...
    gate_flow_coeff...
    part_load_freq...
    osc_Gs osc_ampl osc_tstep T_w T_e...
    rotor_inertia...
    Q_base H_base G_base;

%% % % 
% physical constants
a_g = 9.8; % m/s^2 gravitational acceleration
rho = 1000; % kg/m^3 --- water density

%% % % Forebay and tailrace parameters
% forebay parameters
z_fnorm = 539; % (m) forebay operating level
z_fmax = 540; % (m)forebay max level
z_fmin = 500; % (m)forebay min level
z_forebay = z_fnorm;
z_t1 = 319; % (m)maynskaya GES forebay 1
z_t2 = 327; % (m) maynskaya GES forebay 2
z_t3 = 331; % (m)maynskaya GES forebay 3
z_mayn = 324; % (m) maynskaya GES forebay operating level
z_turbine = 314; % (m) turbine height

%% % % Runner parameters
d_runner = 6.77; % (m) runner diameter, 
r_runner = d_runner/2; % (m) runner radius
runner_mass = 153.7*10^3; % (kg) - runner mass
d_shaft = 1.94; % (m) shaft diameter
r_shaft = d_shaft/2; % (m) shaft radius
l_shaft = 7.17; % (m) shaft length
m_shaft = 95.7*10^3; % (kg) shaft mass
shaft_inertia = m_shaft*(r_shaft^2)/2;
% Rotor inertia approximation for solid disc with radius R
runner_inertia = runner_mass/2*(d_runner/2)^2;% (kg*m^2)
complete_inertia = rotor_inertia+shaft_inertia+runner_inertia;
%% % % % % Penstock parameters % % % % % % %
roughness_height = 10^(-3); % (m) concrete roughness height
% solve for Darcy friction factor in case of hughly tubulent flow
% Re=v_max*d_penstock/nu_w; % Reynolds number, Re<2000 - laminar, 2000<Re<10000 transient,
% Re>10000 - turbulent
% R_star = 1/sqrt(8)*(Re*)
% f_p = fzero(@(x)(1/sqrt(x)+2*log10(2.51/(Re*sqrt(x))*(roughness_height/(3.7*d_penstock)+1))),0.012);
d_penstock = 7.5; % (m) penstock diameter
A_penstock = pi*(d_penstock^2)/4;
f_p = 0.013;
sound_speed = 1400; % m/s
L_penstock = 212; % (m) Penstock length

% turbine head and flow parameters
turbine_characteristics; % turbine universal characteristic
H_work = 194; %m net hydraulic head, operating mode
H_min = 0;
H_max = 250; %(m) max turbine head 220 
Q_max = 358; % m^3/s max turbine water flow
Q_max_l = 358*10^3; % l/s

% wicket gates parameters
G_min = 1;
G_max = 58;
gate_flow_coeff = 0.55;


%% turbine model time constants
Power_max = 700*10^6; % max turbine power
T_w =L_penstock*Q_max/(a_g*A_penstock*H_work);
T_e =2*L_penstock/sound_speed;

%% base values 
H_base = 200; % m
% Q_base = S_base/(H_base*rho*a_g);
G_base = G_max;
Q_base = G_base*gate_flow_coeff*sqrt(H_base);
% G_base = Q_base/(gate_flow_coeff*sqrt(H_base));

%% %  turbine oscillations due to the vortex rope  % % % % % % % % % % %
%  corresponding to (precessing?) vortex rope
% powers=[0,100,200, 300, 350, 400, 500,550, 600, 650, 700];
% osc_Gs = [0,8.6,17.2,25.8,30.1,34.4,43, 47.3,51.6,55.9,60];
% amplitudes in m
% bep corresponds to gate G=33

% powers = [0,100,200,250,400,480,560,650,750];
osc_Gs = [0,9,  15, 18, 27, 32, 37, 42.3,47.3];
osc_ampl = [2,3,3.5,4,14,8,8,0,6]/2;
% osc_Gs    =[0,4,8 ,11,15,19,26,27,30,32,34,50];
% osc_heads =[0,3,3 ,8 ,12,11,9 ,4 ,0 ,0 ,0,0];
% frequency
% zone 1:0.1MPa
% zone 2:0.4-1.2Hz, 
% f_osc = 0.5; %Hz
% % % % % % part load vortex % % % % % % 
part_load_freq = 0.3;
osc_tstep = pi/(2*omega_m_nom*part_load_freq)/4;