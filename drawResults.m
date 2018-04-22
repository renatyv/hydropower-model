function [fig_1,fig_2] = drawResults(t,state,steady_state_1,steady_state_2, plot_electric, plot_hydraulic)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

global Q_base G_base omega_m_nom poles_number gate_flow_coeff d_runner...
    omega_er_base S_base torque_base...
    r_s T_r z_tailrace_const eta_Q_u eta_n_u eta_eta_u;

% 
    set(groot,'defaultLineLineWidth',2);
    
    omega_ms = state(:,1);
    omega_steady = steady_state_1(1);
    omega_pus = omega_ms/omega_m_nom;
    qs = state(:,2);
    q_steady = steady_state_1(2);
    gs = state(:,3);
    g_steady = steady_state_1(3);
    Qs = qs*Q_base;
    Q_steady = q_steady*Q_base;
    pilot_servos = state(:,5);
    pilot_servo_steady = steady_state_1(5);
    psi_ds = state(:,6);
    psi_qs = state(:,7);
    psi_rs = state(:,8);
    psi_rds = state(:,9);
    psi_rqs = state(:,10);
    
%    compute exciter voltage
    [e_qs,e_rqs,e_rds,i_qs,i_ds] = psi_to_E(psi_ds,psi_qs,psi_rs,psi_rds,psi_rqs);
    dPsi_ds = estimateDerivative(psi_ds,t);
    dPsi_qs = estimateDerivative(psi_qs,t);
    v_ds = -dPsi_ds/omega_er_base-omega_pus.*psi_qs-r_s*i_ds;
    v_qs = -dPsi_qs/omega_er_base+omega_pus.*psi_ds-r_s*i_qs;
    dPsi_rs = estimateDerivative(psi_rs,t);
    e_rs=(dPsi_rs*T_r+e_qs);
    
%     compute turbine power
    Turbine_powers = zeros(size(Qs));
    H_turbs = zeros(size(Qs));
    for k=1:length(Turbine_powers)
        [ dq,Turbine_power,H_turb,H_loss] = turbineModel(t(k),gs(k),qs(k),omega_ms(k));
        Turbine_powers(k) = Turbine_power;
        H_turbs(k) = H_turb;
    end
    [ dq,Turbine_power_steady,H_turb_steady] = turbineModel(t(k),g_steady,q_steady,omega_steady);
%     compute Q_i, n_i

%     connected to power network
%     omega_max1 = omega_m_nom*(1+0.2/50);
%     omega_min1 = omega_m_nom*(1-0.2/50);
%     omega_max2 = omega_m_nom*(1+0.4/50);
%     omega_min2 = omega_m_nom*(1-0.4/50);
%     disconnected from power network
    omega_max1 = omega_m_nom*(1+1/50);
    omega_min1 = omega_m_nom*(1-1/50);
    omega_max2 = omega_m_nom*(1+5/50);
    omega_min2 = omega_m_nom*(1-5/50);
%%
    fig_2 = figure(2);
    g_min = min(gs);
    g_max = max(gs);
    plot(gs,omega_ms,...
        g_steady,omega_steady,'go',...
        gs(1),omega_ms(1),'r*')
    title(mat2str(steady_state_1,2));
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
    subplot(3,2,1);
    plot(t,omega_ms,[t(1),t(end)],[omega_steady,omega_steady],'g');
    title_str = strcat('initial:',mat2str(state(1,:),2));
    title(title_str);
    
    
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
    ylabel('omega_m');
    
    subplot(3,2,2);
    active_powers = (v_ds.*i_ds+v_qs.*i_qs)*S_base/10^6;
    reactive_powers = (v_qs.*i_ds-v_ds.*i_qs)*S_base/10^6;
    generator_torques = (psi_ds.*i_qs-psi_qs.*i_ds)*torque_base;
    generator_powers_pu = omega_ms.*generator_torques/10^6;
    plot(t,active_powers,t,reactive_powers,t,Turbine_powers/10^6,t,generator_powers_pu);
    legend('active load','reactive load','turbine','generator');
    xlabel('t');
    ylabel('power, MW');

    if plot_hydraulic
        subplot(3,2,4);
        % % draw universal characteristic
        contour(7*eta_Q_u,eta_n_u,eta_eta_u,...
            [0.1,0.3,0.4,0.5,0.6,0.7,0.8,0.84,0.88,0.90,0.91,0.92],'ShowText','On');
        % % draw trajectory
        Q_is=1000*Qs./(d_runner^2*sqrt(H_turbs));
        Q_i_steady = 1000*Q_steady./(d_runner^2*sqrt(H_turb_steady));
        ns=60*omega_ms/(2*pi);
        n_steady = 60*omega_steady/(2*pi);
        n_is=ns*d_runner./sqrt(H_turbs);
        n_i_steady = n_steady*d_runner./sqrt(H_turb_steady);
        hold on;
        plot(Q_is,n_is,'b',Q_is(1),n_is(1),'b*',Q_i_steady,n_i_steady,'go');
        hold off;
        axis([0,700,50,100]);
        xlabel('Q_i');
        ylabel('n_i');
        
        subplot(3,2,3);
        H_base = 200;
        p1 = plot(t,gs,'b',...
        t,pilot_servos,'g',...
        t,qs,'r',...
        t,H_turbs/H_base,'k');
        xlabel('t');
        ylabel('p.u.');
        hold on;
        plot([t(1);t(end)],[g_steady;g_steady],'b--',...
        [t(1);t(end)],[pilot_servo_steady;pilot_servo_steady],'g--',...
        [t(1);t(end)],[q_steady;q_steady],'r--',...
        [t(1);t(end)],[H_turb_steady/H_base;H_turb_steady/H_base],'k--');
        legend([p1],'g','pilot','q','h_{turb}','Location','northeastoutside');
        hold off;
        
    end
       
    if plot_electric
        subplot(3,2,5);
        plot(t,i_ds,'r',t,i_qs,'g',t,v_ds,'b',t,v_qs,'k');
        legend('i_d', 'i_q','v_d','v_q','Location','northeastoutside');
        xlabel('t');
        
        subplot(3,2,6);
        plot(t,sqrt(v_qs.^2+v_ds.^2),t,e_rs);
        legend('sqrt(v_d^2+v_q^2)','e_r');
        xlabel('t');
    end
end

