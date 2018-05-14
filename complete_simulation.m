clear variables;

%% % % configure simulation
integration_stopper = true;
enable_saturation = false;
use_dead_zone = false;
number_of_simulations = 1;
t_max = 20.0;
sim_maxstep = 0.01;

%% configuration

plot_results = true;

%% models parameters
gen_model = GenModel();
turb_model = TurbineModel1();
exciter_model = ExciterModelAC4A();
gov_model = GovernerModel1();
% gov_model = GovernerModelSSHG();
% gov_model = ConstantGoverner();
load_model = LoadModelPQ();
% T_m =complete_inertia*omega_m_nom^2/Power_max;
% % % initial frequency in radian/s, electrical radian/s, HZ and rpm

%% compute and print initial state
[steady_state_1,steady_state_2,N_turb_steady] =...
    get_steady_state(gov_model,gen_model,turb_model,exciter_model,load_model);
disp('steady state 1');
printState(0,steady_state_1,gen_model,turb_model,load_model,gov_model,exciter_model);
disp('steady state 2');
printState(0,steady_state_2,gen_model,turb_model,load_model,gov_model,exciter_model);

%% initialize constant power models
%% TODO implement class ConstantTorqueTurbineModel isntead
turb_model.constant_turbine_power = N_turb_steady;
turb_model.constant_turbine_speed = steady_state_1(1);
gen_model.constant_torque = -turb_model.constant_turbine_power/turb_model.constant_turbine_speed;

%% compute jacobian and check local stability
% assert(max(abs(aut_model(steady_state)))<10^-3,'state is not steady');
model = @(t,state,enable_saturation,use_dead_zone)...
    full_model(t,state,enable_saturation,use_dead_zone,gov_model,gen_model,turb_model,exciter_model,load_model);

aut_model_nosat_nodz =@(s)(model(0,s,false,false));
% minimodel = @(x)(aut_model_nosat_nodz(x)'*aut_model_nosat_nodz(x)); 

assert(max(abs(aut_model_nosat_nodz(steady_state_1)))<10^-3,'state 1 is not steady');
Jac1 = NumJacob(aut_model_nosat_nodz,steady_state_1');

assert(max(abs(aut_model_nosat_nodz(steady_state_2)))<10^-3,'state 2 is not steady');
Jac2 = NumJacob(aut_model_nosat_nodz,steady_state_2');
% disp([eig(Jac1),eig(Jac2)]);
%% simulation
time = [0, t_max];
sim_model_1 = @(t,state)(model(t,state,enable_saturation,use_dead_zone));
for k=1:number_of_simulations
    max_distance = 0.05;
    initial_state = generate_state_near(gov_model,gen_model,turb_model,load_model,steady_state_1,max_distance);
%     initial_state=steady_state_1;
    fprintf('initial state for simulation %d\n',k);
    printState(0,initial_state,gen_model,turb_model,load_model,gov_model);
    options = odeset('MaxStep',sim_maxstep);
    if integration_stopper
        options = odeset('MaxStep',sim_maxstep,'Events',@stop_integration_event);
    end
%     options = odeset('MaxStep',sim_maxstep);
    [t, state] = ode15s(sim_model_1, time, initial_state,options);
    [fig_1,fig_2] =...
        drawResults(t,state,steady_state_1,steady_state_2,gen_model,turb_model,gov_model);
    % save sim results to files
    file_name = sprintf('%.0fMW_load%d',load_model.P_active0/10^6,k);
    saveas(fig_1,sprintf('sim_results/all_%s.png',file_name));
    saveas(fig_2,sprintf('sim_results/pp_%s.png',file_name));
    filename = sprintf('sim_results/data_%s',file_name);
%     save everything except fig_1, fig_2
    save(filename,'-regexp','^(?!(fig_1|fig_2)$).');
end

%% complete model 
function [dstate] = full_model(t,state,enable_saturation,use_dead_zone,gov_model,gen_model,turb_model,exciter_model,load_model)
    [omega_pu,q,g,governer_state,psi,exciter_state] = parseState(state,gov_model.state_size,exciter_model.state_size);
    omega_m = omega_pu*gen_model.omega_m_nom;

    [ dq,Turbine_power,~,~] = turb_model.model(t,g,q,omega_m);
    Turbine_torque = Turbine_power/omega_m;

    [~,~,~,i_q,i_d] = gen_model.psi_to_E(psi);
    [v_d,v_q] = load_model.model(t,i_d,i_q,omega_m,gen_model.omega_m_nom,gen_model.S_base);
    [e_r,dexciter_state] = exciter_model.model(v_q,v_d,exciter_state,enable_saturation);
    
    [ dpsi,Electric_torque] =...
            gen_model.model(psi,v_d,v_q,e_r,omega_m);
    complete_inertia = gen_model.rotor_inertia+turb_model.runner_inertia;
    domega_m = (Turbine_torque+Electric_torque)/complete_inertia;
    [dg,dgoverner_state] =...
        gov_model.model(g,governer_state,omega_m,domega_m,enable_saturation,use_dead_zone);
    dstate = constructState(domega_m,dq,dg,dgoverner_state,dpsi,dexciter_state);
end