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
    N_turb_const omega_m_const T_m K_f;


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
K_f = 0.7;
% K_dOmega must be small, otherwise unstable
K_dOmega=0.0;
fprintf('governer K_p=%.1f, K_i=%.1f, K_d=%.1f, K_f=%.1f\n',PID_Kp,PID_Ki,K_dOmega,K_f);

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
P_active0 = 600*10^6;
Q_reactive0 = P_active0/cos(phi_1)*sin(phi_1);
% N_gen = -300*10^6; %W, generator power
constant_exciter = false;

use_simple_gateflow_model = false;
use_constant_turbine_efficiency = false;
turbine_const_efficiency = 0.8;
simulate_vortex_rope_oscillations = false;
plot_results = true;


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
generatorParameters;
turbineParameters;
complete_inertia=runner_inertia+rotor_inertia;
T_m =complete_inertia*omega_m_nom^2/Power_max;
governerParameters;
% % % initial frequency in radian/s, electrical radian/s, HZ and rpm

%% compute and print initial state
[steady_state_1,steady_state_2,N_turb_steady,e_r_1,e_r_2] =...
    get_steady_state(P_active0,Q_reactive0);
disp('steady state 1');
printState(steady_state_1);
disp('steady state 2');
printState(steady_state_2);

%% for constant turbine power models set 
N_turb_const = N_turb_steady;
omega_m_const = steady_state_1(1);

%% compute jacobian and check local stability
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
    max_distance = 0.05;
    initial_state = generate_state_near(steady_state_1,max_distance);
%     initial_state=steady_state_1;
    fprintf('initial state for simulation %d\n',k);
    printState(initial_state);
    options = odeset('MaxStep',sim_maxstep);
    if integration_stopper
        options = odeset('MaxStep',sim_maxstep,'Events',@stop_integration_event);
    end
%     options = odeset('MaxStep',sim_maxstep);
    [t, state] = ode15s(sim_model_1, time, initial_state,options);
    [fig_1,fig_2] =...
        drawResults(t,state,steady_state_1,steady_state_2);
    % save sim results to files
    file_name = sprintf('%.0fMW_load%d',P_active0/10^6,k);
    saveas(fig_1,sprintf('sim_results/all_%s.png',file_name));
    saveas(fig_2,sprintf('sim_results/pp_%s.png',file_name));
    filename = sprintf('sim_results/data_%s',file_name);
%     save everything except fig_1, fig_2
    save(filename,'-regexp','^(?!(fig_1|fig_2)$).');
end

% if plot_results && (number_of_simulations < 1)
%     load 'sim_results/data_1';
%     drawResults(t,state,steady_state_1,steady_state_2);
% end

%% complete model 
function [dstate] = full_model(t,state,e_r_const,enable_saturation,use_dead_zone)
global complete_inertia poles_number;
    [omega_m,q,g,governer_state,psi,exciter_state] = parseState(state);
    
    [ dq,Turbine_power,~,~] = turbineModel(t,g,q,omega_m);
    Turbine_torque = Turbine_power/omega_m;

    [~,~,~,i_q,i_d] = psi_to_E(psi);
    [v_d,v_q] = loadModel(t,i_d,i_q,omega_m);
    [e_r,dexciter_state] = exciterModelPI(v_q,v_d,exciter_state,enable_saturation,e_r_const);
    
    omega_er = omega_m*poles_number; % electrical radians
    [ dpsi,Electric_torque] =...
            generatorModel(psi,v_d,v_q,e_r,omega_er);
    domega_m = (Turbine_torque+Electric_torque)/complete_inertia;
    [dg,dgoverner_state] =...
        governerModel(g,governer_state,omega_m,domega_m,enable_saturation,use_dead_zone);
    dstate = constructState(domega_m,dq,dg,dgoverner_state,dpsi,dexciter_state);
end