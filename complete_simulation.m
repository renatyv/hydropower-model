clear variables;
clear global;
global constant_generator_torque...
    constant_exciter constant_governer...
    P_active0 Q_reactive0...
    load_mode...
    exciter_PID_Ki exciter_PID_Kp;

%% % % configure simulation
integration_stopper = false;
enable_saturation = false;
number_of_simulations = 1;
t_max = 20.0;
sim_maxstep = 0.01;

%% exciter governer parameters
exciter_PID_Ki=2;
exciter_PID_Kp=10;
fprintf('exciter K_p=%.1f, K_i=%.1f\n',exciter_PID_Kp,exciter_PID_Ki);

%% 0 -- constant MVA, 22 (a=b=2)-- constant impedance, 12 (a=1,b=2)
load_mode = 22;

%% configuration
constant_governer = false;
use_dead_zone = false;

constant_generator_torque = false;
phi_1 = acos(0.9);
P_active0 = 600*10^6;
Q_reactive0 = P_active0/cos(phi_1)*sin(phi_1);
% N_gen = -300*10^6; %W, generator power
constant_exciter = false;

plot_results = true;

%% models parameters
gen_model = GenModel();
turb_model = TurbineModel1();
% T_m =complete_inertia*omega_m_nom^2/Power_max;
gov_model = GovernerModel1();
% % % initial frequency in radian/s, electrical radian/s, HZ and rpm

%% compute and print initial state
[steady_state_1,steady_state_2,N_turb_steady,e_r_1,e_r_2] =...
    get_steady_state(gov_model,gen_model,turb_model,P_active0,Q_reactive0);
disp('steady state 1');
printState(steady_state_1,gen_model,turb_model);
disp('steady state 2');
printState(steady_state_2,gen_model,turb_model);

%% for constant turbine power models set 
turb_model.constant_turbine_power = N_turb_steady;
turb_model.constant_turbine_speed = steady_state_1(1);

%% compute jacobian and check local stability
% assert(max(abs(aut_model(steady_state)))<10^-3,'state is not steady');
model = @(t,state,e_r_const,enable_saturation,use_dead_zone)...
    full_model(t,state,e_r_const,enable_saturation,use_dead_zone,gov_model,gen_model,turb_model);

aut_model_nosat_nodz_ss1 =@(s)(model(0,s,e_r_1,false,false));
Jac1 = NumJacob(aut_model_nosat_nodz_ss1,steady_state_1');
assert(max(abs(aut_model_nosat_nodz_ss1(steady_state_1)))<10^-3,'state 1 is not steady');

aut_model_nosat_nodz_ss2 =@(s)(model(0,s,e_r_2,false,false));
Jac2 = NumJacob(aut_model_nosat_nodz_ss2,steady_state_2');
assert(max(abs(aut_model_nosat_nodz_ss2(steady_state_2)))<10^-3,'state 2 is not steady');
% disp([eig(Jac1),eig(Jac2)]);
%% simulation
time = [0, t_max];
sim_model_1 = @(t,state)(model(t,state,e_r_1,enable_saturation,use_dead_zone));
for k=1:number_of_simulations
    max_distance = 0.05;
    initial_state = generate_state_near(gov_model,gen_model,turb_model,steady_state_1,max_distance);
%     initial_state=steady_state_1;
    fprintf('initial state for simulation %d\n',k);
    printState(initial_state,gen_model,turb_model);
    options = odeset('MaxStep',sim_maxstep);
    if integration_stopper
        options = odeset('MaxStep',sim_maxstep,'Events',@stop_integration_event);
    end
%     options = odeset('MaxStep',sim_maxstep);
    [t, state] = ode15s(sim_model_1, time, initial_state,options);
    [fig_1,fig_2] =...
        drawResults(t,state,steady_state_1,steady_state_2,gen_model,turb_model);
    % save sim results to files
    file_name = sprintf('%.0fMW_load%d',P_active0/10^6,k);
    saveas(fig_1,sprintf('sim_results/all_%s.png',file_name));
    saveas(fig_2,sprintf('sim_results/pp_%s.png',file_name));
    filename = sprintf('sim_results/data_%s',file_name);
%     save everything except fig_1, fig_2
    save(filename,'-regexp','^(?!(fig_1|fig_2)$).');
end

%% complete model 
function [dstate] = full_model(t,state,e_r_const,enable_saturation,use_dead_zone,gov_model,gen_model,turb_model)
    [omega_m,q,g,governer_state,psi,exciter_state] = parseState(state);
    
    [ dq,Turbine_power,~,~] = turb_model.model(t,g,q,omega_m);
    Turbine_torque = Turbine_power/omega_m;

    [~,~,~,i_q,i_d] = gen_model.psi_to_E(psi);
    [v_d,v_q] = loadModel(t,i_d,i_q,omega_m,gen_model.S_base,gen_model.omega_m_nom);
    [e_r,dexciter_state] = exciterModelPI(v_q,v_d,exciter_state,enable_saturation,e_r_const);
    
    [ dpsi,Electric_torque] =...
            gen_model.model(psi,v_d,v_q,e_r,omega_m);
    complete_inertia = gen_model.rotor_inertia+turb_model.runner_inertia;
    domega_m = (Turbine_torque+Electric_torque)/complete_inertia;
    [dg,dgoverner_state] =...
        gov_model.model(g,governer_state,omega_m,domega_m,enable_saturation,use_dead_zone);
    dstate = constructState(domega_m,dq,dg,dgoverner_state,dpsi,dexciter_state);
end