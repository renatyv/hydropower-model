clear variables;
clear global;
global use_simple_gateflow_model use_constant_turbine_efficiency...
    simulate_vortex_rope_oscillations...
    d_runner...
    a_g rho...
    omega_m_nom poles_number...
    gate_flow_coeff...
    PID_Kp PID_Ki K_dOmega...
    turbine_eta_m turbine_const_efficiency...
    complete_inertia runner_inertia rotor_inertia...
    constant_turbine_torque constant_generator_torque...
    omega_er_base...
    z_tailrace_const z_forebay...
    e_r_const constant_exciter constant_governer...
    P_active0 Q_reactive0...
    load_mode...
    Q_base G_base S_base torque_base...
    exciter_PID_Ki exciter_PID_Kp...
    T_r r_s use_dead_zone Power_max...
    use_integrator...
    N_turb_const omega_m_const T_m k_feedback;


%% % % configure simulation
% number_of_simulations = 500;
number_of_simulations = 100;
t_max = 20.0;
sim_maxstep = 0.01;
S_base = 640*10^6; % Base power, Watt, affects simulation stability

%% turbine governer parameters
PID_Kp=30;
PID_Ki=2;
k_feedback = 0.7;
% K_dOmega must be small, otherwise unstable
K_dOmega=0.0;


%% exciter governer parameters
exciter_PID_Ki=2;
exciter_PID_Kp=10;

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
[steady_state_1,steady_state_2,N_turb_steady,exciter_state_1,exciter_state_2] =...
    get_steady_state(P_active0,Q_reactive0);
steady_state = steady_state_2;
e_r_const = exciter_state_2;
% steady_state = steady_state_1;
% e_r_const = exciter_state_1;
N_turb_const = N_turb_steady;
omega_m_const = steady_state(1);
%% check that initial state is steady 
aut_model =@(s)(full_model(0,s));
disp('steady state');
printState(steady_state);
assert(max(abs(aut_model(steady_state)))<10^-3,'state is not steady');
%% compute jacobian 

if use_dead_zone
    use_dead_zone = false;
    Jac1 = NumJacob(aut_model,steady_state_1');
    Jac2 = NumJacob(aut_model,steady_state_2');
    use_dead_zone = true;
else
    Jac1 = NumJacob(aut_model,steady_state_1');
    Jac2 = NumJacob(aut_model,steady_state_2');
end
disp([eig(Jac1),eig(Jac2)]);
%% simulation
% t=[0];
% state = [initial_state];
time = [0, t_max];
for k=1:number_of_simulations
    initial_state = generate_state_near(steady_state);
%     initial_state = [14.7524    0.1055    0.1220    0.1806    0.1940   -0.9569...
%         -0.1584   -0.9454   -1.2136   -0.1122   -0.6520];
%     initial_state = steady_state;
    printState(initial_state);
    options = odeset('MaxStep',sim_maxstep,'Events',@stop_integration_event);
%     options = odeset('MaxStep',sim_maxstep);
    [t, state] = ode15s(@full_model, time, initial_state,options);
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
function [dstate] = full_model(t,state)
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
    [e_r,dexciter_state] = exciterModelPI(v_q,v_d,exciter_state);
    [ dpsi_d, dpsi_q, dpsi_r,dpsi_rd,dpsi_rq,Electric_torque] =...
            generatorModelDAN(psi_d,psi_q,psi_r,psi_rd,psi_rq,v_d,v_q,e_r,omega_er);
    dOmega_m = (Turbine_torque+Electric_torque)/complete_inertia;
    [dg,dPID_i,dpilot_servo] =...
        governerModel(g,PID_i,pilot_servo,omega_m,dOmega_m);
    dstate=[dOmega_m;...
        dq;dg;dPID_i;dpilot_servo;...
        dpsi_d; dpsi_q; dpsi_r;dpsi_rd;dpsi_rq;...
        dexciter_state];
end