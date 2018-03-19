function [fig_1,fig_2] = drawResults(t,state,steady_state_1,steady_state_2, plot_electric, plot_hydraulic)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
global Q_base G_base omega_m_nom poles_number gate_flow_coeff d_runner...
    omega_er_base S_base torque_base...
    r_s T_r z_tailrace0 eta_Q_u eta_n_u eta_eta_u;

% 
    set(groot,'defaultLineLineWidth',2);
    
    omega_ms = state(:,1);
    omega_steady = steady_state_1(1);
    omega_ers = omega_ms*poles_number;
    omega_pus = omega_ms/omega_m_nom;
    qs = state(:,2);
    q_steady = steady_state_1(2);
    gs = state(:,3);
    g_steady = steady_state_1(3);
    Gs = gs*G_base;
    Qs = qs*Q_base;
    G_steady = g_steady*G_base;
    Q_steady = q_steady*Q_base;
    PID_is = state(:,4);
    PID_i_steady = steady_state_1(4);
    pilot_servos = state(:,5);
    pilot_servo_steady = steady_state_1(5);
    psi_ds = state(:,6);
    psi_d_steady_1 = steady_state_1(6);
    psi_d_steady_2 = steady_state_2(6);
    psi_qs = state(:,7);
    psi_q_steady_1 = steady_state_1(7);
    psi_q_steady_2 = steady_state_2(7);
    psi_rs = state(:,8);
    psi_r_steady_1 = steady_state_1(8);
    psi_r_steady_2 = steady_state_2(8);
    psi_rds = state(:,9);
    psi_rd_steady_1 = steady_state_1(9);
    psi_rd_steady_2 = steady_state_2(9);
    psi_rqs = state(:,10);
    psi_rq_steady_1 = steady_state_1(10);
    psi_rq_steady_2 = steady_state_2(10);
    exciter_states = state(:,11);
    
%    compute exciter voltage
    [e_qs,e_rqs,e_rds,i_qs,i_ds] = psi_to_E(psi_ds,psi_qs,psi_rs,psi_rds,psi_rqs);
    dPsi_ds = estimateDerivative(psi_ds,t);
    dPsi_qs = estimateDerivative(psi_qs,t);
    v_ds = -dPsi_ds/omega_er_base-omega_pus.*psi_qs-r_s*i_ds;
    v_qs = -dPsi_qs/omega_er_base+omega_pus.*psi_ds-r_s*i_qs;
    dPsi_rs = estimateDerivative(psi_rs,t);
    e_rs=(dPsi_rs*T_r+e_qs);
    
%     compute turbine power
    turbine_powers = zeros(size(Qs));
    H_turbs = zeros(size(Qs));
    for k=1:length(turbine_powers)
        [ dq,turbine_torque,H_turb] = turbineModel(t(k),gs(k),qs(k),omega_ms(k),z_tailrace0);
        turbine_powers(k) = turbine_torque*omega_ms(k);
        H_turbs(k) = H_turb;
    end
    [ dq,turbine_torque_steady,H_turb_steady] = turbineModel(t(k),g_steady,q_steady,omega_steady,z_tailrace0);
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

    fig_2 = figure(2);
    g_min = min(gs);
    g_max = max(gs);
    plot(gs,omega_ms,...
        g_steady,omega_steady,'go',...
        gs(1),omega_ms(1),'r*')
    hold on;
%     if max(omega_ms)>omega_max1
        plot([g_min;g_max],[omega_max1,omega_max1],'b--');
%     end
%     if min(omega_ms)<omega_min1
        plot([g_min;g_max],[omega_min1,omega_min1],'b--');
%     end
    if max(omega_ms)>omega_max2
        plot([g_min;g_max],[omega_max2,omega_max2],'b--');
    end
    if min(omega_ms)<omega_min2
        plot([g_min;g_max],[omega_min2,omega_min2],'b--');
    end
    hold off;
    xlabel('g');
    ylabel('omega_m');

    fig_1 = figure(1);
    set(fig_1, 'Position', [10, 600, 800, 900])
    subplot(3,2,1);
    plot(t,omega_ms,[t(1),t(end)],[omega_steady,omega_steady],'g');
    hold on;
    if max(omega_ms)>omega_max1
        plot([t(1);t(end)],[omega_max1,omega_max1],'b--');
    end
    if min(omega_ms)<omega_min1
        plot([t(1);t(end)],[omega_min1,omega_min1],'b--');
    end
    if max(omega_ms)>omega_max2
        plot([t(1);t(end)],[omega_max2,omega_max2],'b--');
    end
    if min(omega_ms)<omega_min2
        plot([t(1);t(end)],[omega_min2,omega_min2],'b--');
    end
    hold off;
    xlabel('t');
    ylabel('omega_m');
    
%     figure(2);
    subplot(3,2,2);
    active_powers = (v_ds.*i_ds+v_qs.*i_qs)*S_base/10^6;
    reactive_powers = (v_qs.*i_ds-v_ds.*i_qs)*S_base/10^6;
    turbine_powers_pu = turbine_powers/10^6;
    generator_torques = (psi_ds.*i_qs-psi_qs.*i_ds)*torque_base;
    generator_powers_pu = omega_ms.*generator_torques/10^6;
    plot(t,active_powers,t,reactive_powers,t,turbine_powers_pu,t,generator_powers_pu);
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
        
%         figure(5);
%         subplot(3,2,6);
%         plot(t,H_turbs,t,Qs);
%         xlabel('t, s');
%         legend('Turbine head, m', 'discharge, m^3/s');
    end
       
    if plot_electric
%         figure(7);
%         plot(t,sqrt(v_qs.^2+v_ds.^2));
%         legend('sqrt(v_d^2+v_q^2)');
%         xlabel('t');
        subplot(3,2,5);
%         figure(8);
        plot(t,i_ds,'r',t,i_qs,'g',t,v_ds,'b',t,v_qs,'k');
        legend('i_d', 'i_q','v_d','v_q','Location','northeastoutside');
        xlabel('t');
        
        subplot(3,2,6);
%         figure(8);
        plot(t,sqrt(v_qs.^2+v_ds.^2));
        ylabel('sqrt(v_d^2+v_q^2)');
        xlabel('t');

%         figure(9);
%         plot(t,e_qs,t,e_rs);
%         legend('e_q','e_r');
%         xlabel('t');
        
%         figure(10);
%         plot(t,psi_ds,t,psi_qs,t,psi_rs,t,psi_rds,t,psi_rqs);
%         legend('\psi_d','\psi_q','\psi_r','\psi_{rd}','\psi_{rq}');
%         xlabel('t');
    end
end

