clear variables;
clear global;
global use_simple_gateflow_model use_constant_turbine_efficiency...
    simulate_vortex_rope_oscillations...
    omega_m_nom...
    PID_Kp PID_Ki K_dOmega...
    turbine_const_efficiency...
    complete_inertia runner_inertia rotor_inertia...
    constant_turbine_torque constant_generator_torque...
    z_tailrace_const z_forebay...
    constant_exciter constant_governer...
    P_active0 Q_reactive0...
    load_mode...
    S_base...
    exciter_PID_Ki exciter_PID_Kp...
    Power_max...
    N_turb_const omega_m_const T_m k_feedback;


%% % % configure simulation
integration_stopper = false;
enable_saturation = false;
number_of_simulations = 1;
t_max = 20.0;
sim_maxstep = 0.01;
S_base = 640*10^6; % Base power, Watt, affects simulation stability

%% turbine governer parameters
PID_Kp=30;
PID_Ki=2;
k_feedback = 0.7;
% K_dOmega must be small, otherwise unstable
K_dOmega=0.0;
fprintf('governer K_p=%.1f, K_i=%.1f, K_d=%.1f, k_feedback=%.1f\n',PID_Kp,PID_Ki,K_dOmega,k_feedback);

%% exciter governer parameters
exciter_PID_Ki=2;
exciter_PID_Kp=10;
fprintf('exciter K_p=%.1f, K_i=%.1f\n',exciter_PID_Kp,exciter_PID_Ki);

%% 0 -- constant MVA, 22 (a=b=2)-- constant impedance, 12 (a=1,b=2)
load_mode = 22;

%% configuration
constant_turbine_torque = false;
constant_governer = false;
use_dead_zone = false;

constant_generator_torque = false;
phi_1 = acos(0.9);
P_active0 = 250*10^6;
Q_reactive0 = P_active0/cos(phi_1)*sin(phi_1);
% N_gen = -300*10^6; %W, generator power
constant_exciter = false;

use_simple_gateflow_model = true;
use_constant_turbine_efficiency = true;
turbine_const_efficiency = 0.8;
simulate_vortex_rope_oscillations = false;
plot_results = true;
plot_electric = true;
plot_hydraulic = true;


%% initialize parameters
rpm_nom = 142.8; % revolutions per minute nomial frequency
omega_m_nom = rpm_nom/60*2*pi; %(rad/s), mechanical frequency of the rotor
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
z_tailrace_const = z_t3; % tailrace

%% models parameters
generatorParametersDAN;
turbineParameters;
complete_inertia=runner_inertia+rotor_inertia;
T_m =complete_inertia*omega_m_nom^2/Power_max;
governerParameters;
exciterParameters;
% % % initial frequency in radian/s, electrical radian/s, HZ and rpm

%% initial state
[steady_state_1,steady_state_2,N_turb_steady,e_r_1,e_r_2] =...
    get_steady_state(P_active0,Q_reactive0);
disp('steady state 1');
printState(steady_state_1);
disp('steady state 2');
printState(steady_state_2);

N_turb_const = N_turb_steady;
omega_m_const = steady_state_1(1);

%% compute jacobian 
% assert(max(abs(aut_model(steady_state)))<10^-3,'state is not steady');
aut_model_nosat_nodz_ss1 =@(s)(full_model(0,s,e_r_1,false,false));
Jac1 = NumJacob(aut_model_nosat_nodz_ss1,steady_state_1');
assert(max(abs(aut_model_nosat_nodz_ss1(steady_state_1)))<10^-3,'state 1 is not steady');

aut_model_nosat_nodz_ss2 =@(s)(full_model(0,s,e_r_2,false,false));
Jac2 = NumJacob(aut_model_nosat_nodz_ss2,steady_state_2');
assert(max(abs(aut_model_nosat_nodz_ss2(steady_state_2)))<10^-3,'state 2 is not steady');
% disp([eig(Jac1),eig(Jac2)]);
%% simulation
time = [0, t_max];
sim_model_1 = @(t,state)(full_model(t,state,e_r_1,enable_saturation,use_dead_zone));
for k=1:number_of_simulations
    initial_state = generate_state_near(steady_state_1);
    fprintf('initial state for simulation %d\n',k);
    printState(initial_state);
    options = odeset('MaxStep',sim_maxstep);
    if integration_stopper
        options = odeset('MaxStep',sim_maxstep,'Events',@stop_integration_event);
    end
%     options = odeset('MaxStep',sim_maxstep);
    [t, state] = ode15s(sim_model_1, time, initial_state,options);
    [fig_1,fig_2] =...
        drawResults(t,state,steady_state_1,steady_state_2, plot_electric, plot_hydraulic);
    saveas(fig_1,sprintf('sim_results/all_%d.png',k));
    saveas(fig_2,sprintf('sim_results/phase_p_%d.png',k));
    filename = sprintf('sim_results/data_%d',k);
%     save everything except fig_1, fig_2
    save(filename,'-regexp','^(?!(fig_1|fig_2)$).');
end

if plot_results
    if number_of_simulations < 1
        load 'sim_results/data_1';
    end
    drawResults(t,state,steady_state_1,steady_state_2, plot_electric, plot_hydraulic);
end

%% complete model 
function [dstate] = full_model(t,state,e_r_const,enable_saturation,use_dead_zone)
global complete_inertia poles_number;
    omega_m = state(1);
    q = state(2);
    g = state(3);
    PID_i = state(4);
    pilot_servo = state(5);
    psi_d = state(6);
    psi_q = state(7);
    psi_r = state(8);
    psi_rd = state(9);
    psi_rq = state(10);
    exciter_state = state(11);
    omega_er = omega_m*poles_number; % electrical radians
    
    [ dq,Turbine_power,~,~] = turbineModel(t,g,q,omega_m);
    Turbine_torque = Turbine_power/omega_m;

    [~,~,~,i_q,i_d] = psi_to_E(psi_d,psi_q,psi_r,psi_rd,psi_rq);
    [v_d,v_q] = loadModel(t,i_d,i_q,omega_m);
    [e_r,dexciter_state] = exciterModelPI(v_q,v_d,exciter_state,enable_saturation,e_r_const);
    [ dpsi_d, dpsi_q, dpsi_r,dpsi_rd,dpsi_rq,Electric_torque] =...
            generatorModelDAN(psi_d,psi_q,psi_r,psi_rd,psi_rq,v_d,v_q,e_r,omega_er);
    dOmega_m = (Turbine_torque+Electric_torque)/complete_inertia;
    [dg,dPID_i,dpilot_servo] =...
        governerModel(g,PID_i,pilot_servo,omega_m,dOmega_m,enable_saturation,use_dead_zone);
    dstate=[dOmega_m;...
        dq;dg;dPID_i;dpilot_servo;...
        dpsi_d; dpsi_q; dpsi_r;dpsi_rd;dpsi_rq;...
        dexciter_state];
end