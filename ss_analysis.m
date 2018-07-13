clear variables;

%% % % configure simulation
integration_stopper = true;
enable_saturation = false;
use_dead_zone = false;

%% models parameters
gen_model = GenModel();
turb_model = TurbineModel1();
% exciter_model = ExciterModelAC4A();
exciter_model = ExciterPIModel();
% gov_model = GovernerModel1();
gov_model = GovernerModelSSHG();
% gov_model = ConstantGoverner();
% initial active power and coefficient
P_active = 600*10^6;
phi = acos(0.9);

a(1,1)=1;
for k = (1:1:10000)
    gov_model.PID_Kp = 100*rand;
    gov_model.PID_Ki = 100*rand;
    gov_model.K_f = 2*rand;
%     exciter_model.exciter_PID_Kp = 10*rand;
%     exciter_model.exciter_PID_Ki = 10*rand;
    load_modes = [0,22,21];
    load_mode=load_modes(randi([1 length(load_modes)],1,1));
    l1 = max_eigenvalue(600*10^6,load_mode,turb_model,gen_model,exciter_model,gov_model);
    if l1>=0
        continue
    else
        
    end
    l2 = max_eigenvalue(400*10^6,load_mode,turb_model,gen_model,exciter_model,gov_model);
    if l2<0
        continue
    else
        fprintf('600,400, l1=%.2f PID_Kp=%.0f PID_Ki=%.0f K_f=%.1f load_mode=%d\n',...
            l1,gov_model.PID_Kp,gov_model.PID_Ki,gov_model.K_f,load_mode);
    end
    l3 = max_eigenvalue(100*10^6,load_mode,turb_model,gen_model,exciter_model,gov_model);
    if l3>=0
        continue
    else
        disp('parameters found');
        disp(gov_model.PID_Kp);
        disp(gov_model.PID_Ki);
        disp(gov_model.K_f);
        disp(exciter_model.exciter_PID_Ki);
        disp(exciter_model.exciter_PID_Kp);
        disp(load_mode);
    end
end

%% complete model 
function [lambda] = max_eigenvalue(P_active,load_mode,turb_model,gen_model,exciter_model,gov_model)
    phi = acos(0.9);
    load_model = LoadModelPQ(P_active,phi);
    load_model.load_mode = load_mode;
    % T_m =complete_inertia*omega_m_nom^2/Power_max;
    % % % initial frequency in radian/s, electrical radian/s, HZ and rpm

    %% compute and print initial state
    [steady_state_1,~,~] =...
        get_steady_state(turb_model,gov_model,gen_model,exciter_model,load_model);

    %% compute jacobian and check local stability
    % assert(max(abs(aut_model(steady_state)))<10^-3,'state is not steady');
    model = @(t,state,enable_saturation,use_dead_zone)...
        full_model(t,state,enable_saturation,use_dead_zone,...
        turb_model,gov_model,gen_model,exciter_model,load_model);

    aut_model_nosat_nodz =@(s)(model(0,s,false,false));
    % minimodel = @(x)(aut_model_nosat_nodz(x)'*aut_model_nosat_nodz(x)); 

%     assert(max(abs(aut_model_nosat_nodz(steady_state_1)))<10^-3,'state 1 is not steady');
    Jac1 = NumJacob(aut_model_nosat_nodz,steady_state_1);
    lambda = max(real(eig(Jac1)));
end

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