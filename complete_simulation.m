clear variables;

%% Create folders for storing simulation results
SIM_RESULTS_FOLDER = 'sim_results';
if ~exist(SIM_RESULTS_FOLDER,'dir')
    mkdir(SIM_RESULTS_FOLDER)
end

%% % % configure simulation
integration_stopper = true;
% saturation for the governor servos, default false
enable_saturation = false;
% governor dead zone. if true, governor will not react to small changes of
% rotation speed omega near zero, default false
use_dead_zone = false;
% simulate with randomized parameters. number of simulations, default 1
number_of_simulations = 1;
% simulation time, default 30.0
t_max = 30.0;
% max simulation step, default 0.01
sim_maxstep = 0.01;

%% configuration
plot_results = true;

%% initialize models    
% generator model
gen_model = GenModel();
% turbine model
turb_model = TurbineModel1();
% use variable turbine efficiency
turb_model.use_constant_turbine_efficiency = false;
turb_model.simulate_vortex_rope_oscillations = true;
exciter_model = ExciterModelAC4A();
% exciter_model = ExciterPIModel();
% gov_model = GovernerModel1();
gov_model = GovernerModelSSHG();
% gov_model = ConstantGoverner();
% initial active power and coefficient, Watts
P_active = 100*10^6;
% active vs reactive power angle
phi = acos(0.9);
% load model
load_model = LoadModelPQ(P_active,phi);

%% compute and print initial state
[steady_state_1,steady_state_2,N_turb_steady] =...
    get_steady_state(turb_model,gov_model,gen_model,exciter_model,load_model);

disp('steady state 1');
printState(0,steady_state_1,turb_model,gov_model,gen_model,exciter_model,load_model);
disp('steady state 2');
printState(0,steady_state_2,turb_model,gov_model,gen_model,exciter_model,load_model);

%% initialize constant power models
turb_model.constant_turbine_power = N_turb_steady;
turb_model.constant_turbine_speed = steady_state_1(1);
gen_model.constant_torque = -turb_model.constant_turbine_power/turb_model.constant_turbine_speed;

%% compute jacobian and check local stability
model = @(t,state,enable_saturation,use_dead_zone)...
    full_model(t,state,enable_saturation,use_dead_zone,...
    turb_model,gov_model,gen_model,exciter_model,load_model);

aut_model_nosat_nodz =@(s)(model(0,s,false,false));

assert(max(abs(aut_model_nosat_nodz(steady_state_1)))<10^-3,'state 1 is not steady');
Jac1 = NumJacob(aut_model_nosat_nodz,steady_state_1);
disp(eig(Jac1));
%% simulation
time = [0, t_max];
sim_model_1 = @(t,state)(model(t,state,enable_saturation,use_dead_zone));
for k=1:number_of_simulations
    max_distance = 0.05;
    initial_state=steady_state_1;
%     start from some random state near the steady state
%     initial_state = generate_state_near(gov_model,gen_model,turb_model,load_model,...
%         steady_state_1,max_distance);
    fprintf('initial state for simulation %d\n',k);
    printState(0,initial_state,turb_model,gov_model,gen_model,exciter_model,load_model);
    options = odeset('MaxStep',sim_maxstep);
    if integration_stopper
        options = odeset('MaxStep',sim_maxstep,'Events',@stop_integration_event);
    end
    [t, state] = ode15s(sim_model_1, time, initial_state,options);
    [fig_1,fig_2] =...
        drawResults(t,state,steady_state_1,steady_state_2,...
        turb_model,gov_model,gen_model,exciter_model,load_model);
    % save sim results to files
    file_name = sprintf('%.0fMW_load%d',load_model.P_active0/10^6,k);
    saveas(fig_1,sprintf('%s/all_%s.png',SIM_RESULTS_FOLDER,file_name));
    saveas(fig_2,sprintf('%s/pp_%s.png',SIM_RESULTS_FOLDER,file_name));
    filename = sprintf('%s/data_%s',SIM_RESULTS_FOLDER,file_name);
%     save everything except fig_1, fig_2
    save(filename,'-regexp','^(?!(fig_1|fig_2)$).');
end

%% complete model 
function [dstate] = full_model(t,state,enable_saturation,use_dead_zone,...
    turb_model,gov_model,gen_model,exciter_model,load_model)
    [omega_pu,q,g,governer_state,psi,exciter_state] =...
        parseState(state,gov_model.state_size,exciter_model.state_size);
    omega_m = omega_pu*gen_model.omega_m_nom;

    [ dq,Turbine_power,~,~] = turb_model.model(t,g,q,omega_m);
    Turbine_torque = Turbine_power/omega_m;

    [~,~,~,i_q,i_d] = gen_model.psi_to_E(psi);
%     TODO if load model is 22, the steady state is wrong
    [v_d,v_q] = load_model.model(t,i_d,i_q,omega_m,gen_model.omega_m_nom,gen_model.S_base);
    v_ampl = sqrt(v_d^2+v_q^2);
    [e_r,dexciter_state] = exciter_model.model(v_ampl,exciter_state,enable_saturation);
    
    [ dpsi,Electric_torque] =...
            gen_model.model(psi,v_d,v_q,e_r,omega_m);
    complete_inertia = gen_model.rotor_inertia+turb_model.runner_inertia;
    domega_m = (Turbine_torque+Electric_torque)/complete_inertia;
    [dg,dgoverner_state] =...
        gov_model.model(g,governer_state,omega_m,domega_m,enable_saturation,use_dead_zone);
    dstate = constructState(domega_m,dq,dg,dgoverner_state,dpsi,dexciter_state);
end