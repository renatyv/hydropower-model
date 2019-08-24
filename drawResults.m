function [fig_1,fig_2] = drawResults(t,state,steady_state_1,steady_state_2,...
    turb_model,gov_model,gen_model,exciter_model,load_model)
%drawResults draws state variables over time

%% set linewidth for all plots 
    set(groot,'defaultLineLineWidth',2);
    
%% recover intermidiate values from state
        for k=1:length(state(:,1))
        state_k = state(k,:);
        [omega_pu,qs(k,:),gs(k,:),governer_states(k,:),psis(k,:),exciter_states(k,:)] = parseState(state_k,gov_model.state_size,exciter_model.state_size);
        omega_ms(k,:) = omega_pu*gen_model.omega_m_nom;
    end
    [omega_steady_pu,q_steady,g_steady,governer_steady,psis_steady,exciter_steady] = parseState(steady_state_1,gov_model.state_size,exciter_model.state_size);
    omega_steady = omega_steady_pu*gen_model.omega_m_nom;
    
    omega_pus = omega_ms/gen_model.omega_m_nom;
    Qs = qs*turb_model.Q_base;
    Q_steady = q_steady*turb_model.Q_base;
    e_qs = zeros(size(omega_ms));
    e_rqs = zeros(size(omega_ms));
    e_rds = zeros(size(omega_ms));
    i_qs = zeros(size(omega_ms));
    i_ds = zeros(size(omega_ms));
%%  compute stator and exciter voltages using known generaotr model and psi (not using load and exciter models)
    for k=1:length(omega_ms)
        psi = psis(k,:);
        [e_qs(k),e_rqs(k),e_rds(k),i_qs(k),i_ds(k)] = gen_model.psi_to_E(psi);
    end
	psi_ds = psis(:,1);
    psi_qs = psis(:,2);
    psi_rs = psis(:,3);
    dPsi_ds = estimateDerivative(psi_ds,t);
    dPsi_qs = estimateDerivative(psi_qs,t);
    v_ds = -dPsi_ds/gen_model.omega_er_base-omega_pus.*psi_qs-gen_model.r_s*i_ds;
    v_qs = -dPsi_qs/gen_model.omega_er_base+omega_pus.*psi_ds-gen_model.r_s*i_qs;
    dPsi_rs = estimateDerivative(psi_rs,t);
    e_rs=(dPsi_rs*gen_model.T_r+e_qs);
    
%%     compute turbine power and head
    Turbine_powers = zeros(size(Qs));
    H_turbs = zeros(size(Qs));
    for k=1:length(Turbine_powers)
        [ ~,Turbine_powers(k),H_turbs(k),~] = turb_model.model(t(k),gs(k),qs(k),omega_ms(k));
    end
    [ ~,~,H_turb_steady] = turb_model.model(t(k),g_steady,q_steady,omega_steady);
%     compute Q_i, n_i

%% set frequency operating boundaries
%     connected to power network
%     omega_max1 = omega_m_nom*(1+0.2/50);
%     omega_min1 = omega_m_nom*(1-0.2/50);
%     omega_max2 = omega_m_nom*(1+0.4/50);
%     omega_min2 = omega_m_nom*(1-0.4/50);
%     disconnected from power network
    omega_max1 = gen_model.omega_m_nom*(1+1/50);
    omega_min1 = gen_model.omega_m_nom*(1-1/50);
    omega_max2 = gen_model.omega_m_nom*(1+5/50);
    omega_min2 = gen_model.omega_m_nom*(1-5/50);
%% rotor frequecny VS. gate opening
    fig_2 = figure(2);
    g_min = min(gs);
    g_max = max(gs);
    plot(gs,omega_ms,...
        g_steady,omega_steady,'go',...
        gs(1),omega_ms(1),'r*')
    hold on;
    plot([g_min;g_max],[omega_max1,omega_max1],'b--');
    plot([g_min;g_max],[omega_min1,omega_min1],'b--');
    if max(omega_ms)>omega_max2
        plot([g_min;g_max],[omega_max2,omega_max2],'r--');
    end
    if min(omega_ms)<omega_min2
        plot([g_min;g_max],[omega_min2,omega_min2],'r--');
    end
    hold off;
    xlabel('g');
    ylabel('omega_m');

    %%
    fig_1 = figure(1);
    set(fig_1, 'Position', [10, 600, 800, 900]);
    %%  rotor frequency
    subplot(3,2,1);
    plot(t,omega_ms,[t(1),t(end)],[omega_steady,omega_steady],'g');
    hold on;
    plot([t(1);t(end)],[omega_max1,omega_max1],'b--');
    plot([t(1);t(end)],[omega_min1,omega_min1],'b--');
    if max(omega_ms)>omega_max2
        plot([t(1);t(end)],[omega_max2,omega_max2],'r--');
    end
    if min(omega_ms)<omega_min2
        plot([t(1);t(end)],[omega_min2,omega_min2],'r--');
    end
    hold off;
    xlabel('t');
    ylabel('\omega');
    %% Active, reactive, turbine power
    subplot(3,2,2);
    active_powers = (v_ds.*i_ds+v_qs.*i_qs)*gen_model.S_base/10^6;
    reactive_powers = (v_qs.*i_ds-v_ds.*i_qs)*gen_model.S_base/10^6;
    generator_torques = (psi_ds.*i_qs-psi_qs.*i_ds)*gen_model.torque_base;
    generator_powers_pu = omega_ms.*generator_torques/10^6;
    plot(t,active_powers,t,reactive_powers,t,Turbine_powers/10^6,t,generator_powers_pu);
    legend('active load','reactive load','turbine','generator');
    xlabel('t');
    ylabel('power, MW');

    %% phase portrait on universal characteristic
    subplot(3,2,4);
    % % draw universal characteristic
    [eta_Q_u,eta_n_u,eta_eta_u] = TurbineModel1.getCharacteristic();
    contour(eta_Q_u/TurbineModel1.Qi_coeff,eta_n_u,eta_eta_u,...
        [0.1,0.3,0.4,0.5,0.6,0.7,0.8,0.84,0.88,0.90,0.91,0.92],'ShowText','On');
    % % draw trajectory
    Q_is=1000*Qs./(turb_model.d_runner^2*sqrt(H_turbs));
    Q_i_steady = 1000*Q_steady./(turb_model.d_runner^2*sqrt(H_turb_steady));
    ns=60*omega_ms/(2*pi);
    n_steady = 60*omega_steady/(2*pi);
    n_is=ns*turb_model.d_runner./sqrt(H_turbs);
    n_i_steady = n_steady*turb_model.d_runner./sqrt(H_turb_steady);
    hold on;
    plot(Q_is,n_is,'b',Q_is(1),n_is(1),'b*',Q_i_steady,n_i_steady,'go');
    hold off;
    axis([0,700,50,100]);
    xlabel('Q_i');
    ylabel('n_i');

    %% turbin variables
    subplot(3,2,3);
    H_base = 200;
    p1 = plot(t,gs,'b',...
    t,qs,'r',...
    t,H_turbs/H_base,'k');
    xlabel('t');
    ylabel('p.u.');
    hold on;
    plot([t(1);t(end)],[g_steady;g_steady],'b--',...
    [t(1);t(end)],[q_steady;q_steady],'r--',...
    [t(1);t(end)],[H_turb_steady/H_base;H_turb_steady/H_base],'k--');
    legend([p1],'g','q','h','Location','northeastoutside');
    hold off;

    %% stator voltages and currents
    subplot(3,2,5);
    plot(t,i_ds,'r',t,i_qs,'g',t,v_ds,'b',t,v_qs,'k');
    legend('i_d', 'i_q','v_d','v_q','Location','northeastoutside');
    xlabel('t');
    
    %% stator voltage amplitude and exciter voltage
    subplot(3,2,6);
    plot(t,sqrt(v_qs.^2+v_ds.^2),t,e_rs);
    legend('sqrt(v_d^2+v_q^2)','e_r');
    xlabel('t');
end