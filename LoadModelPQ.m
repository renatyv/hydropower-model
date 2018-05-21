classdef LoadModelPQ
    properties
        phi_1;
        P_active0;
        Q_reactive0;
        S_full;
        % 0 -- constant MVA, 22 (a=b=2)-- constant impedance, 12 (a=1,b=2)
        load_mode = 22;
    end
    methods
        function obj = LoadModelPQ(P_active,phi)
            obj.P_active0 = P_active;
            obj.phi_1 = phi;
            obj.S_full = obj.P_active0/cos(obj.phi_1);
            obj.Q_reactive0 = obj.S_full*sin(obj.phi_1);
        end
        
        function [v_d,v_q] = model(this,t,i_d,i_q,omega_m,omega_m_nom,S_base)
        %LOAD_MODEL Implements load models
        % load_resistance0 = S_base/P_active0;
        step_start = 1.0;
        step_end = 5.0;
        p_start = this.P_active0/S_base;
        q_start = this.Q_reactive0/S_base;
%         load ramp
%         p_end = (200*10^6)/S_base;
%         q_end = p_end*sin(acos(0.9))/0.9;
        p_end = p_start;
        q_end = q_start;
        p_active = rramp(p_start,p_end,step_start,step_end,t);
        q_reactive = rramp(q_start,q_end,step_start,step_end,t);

        %% frequency dependance 
        % D_pf = 0..3
        % D_qf = 0..-2
%         D_pf = 0;
%         D_qf = 0;
%         f_ref = omega_m_nom/(2*pi);
%         f = omega_m/(2*pi);
%         delta_f = f-f_ref;
%         p_active = p_active*(1+D_pf*delta_f);
%         q_reactive = q_reactive*(1+D_qf*delta_f);
%         s_full = (this.S_full/S_base)^2;
        %% load model
        if this.load_mode == 0
        %     constant MVA, a=b=0
            v_d = (i_d.*p_active - i_q.*q_reactive)./(i_d.^2 + i_q.^2);
            v_q = (i_q.*p_active + i_d.*q_reactive)./(i_d.^2 + i_q.^2);
        else
            if this.load_mode == 22
        %     constant impedance, a=b=2
                v_d = (i_d.*p_active - i_q.*q_reactive)./(p_active.^2 + q_reactive.^2);
                v_q = (i_q.*p_active + i_d.*q_reactive)./(p_active.^2 + q_reactive.^2);
            else
                p=p_active;
                q=q_reactive;
                v_d1 = -(i_d.^2.*i_q - i_q.*p.^2 + i_q.^3 - i_d.*p.*(i_d.^2 + i_q.^2 - p.^2).^(1/2))/(q.*(i_d.^2 + i_q.^2));
                v_d2 = -(i_d.^2.*i_q - i_q.*p.^2 + i_q.^3 + i_d.*p.*(i_d.^2 + i_q.^2 - p.^2).^(1/2))/(q.*(i_d.^2 + i_q.^2));

                v_q1 = (i_d.*i_q.^2 - i_d.*p.^2 + i_d.^3 + i_q.*p.*(i_d.^2 + i_q.^2 - p.^2).^(1/2))/(q.*(i_d.^2 + i_q.^2));
                v_q2 = (i_d.*i_q.^2 - i_d.*p.^2 + i_d.^3 - i_q.*p.*(i_d.^2 + i_q.^2 - p.^2).^(1/2))/(q.*(i_d.^2 + i_q.^2));
        %         choose same sign as for constant impedance model
                if sign((i_d.*p_active - i_q.*q_reactive)./(p_active.^2 + q_reactive.^2))==sign(v_d1)
                    v_d = v_d1;
                    v_q = v_q1;
                else
                    v_d = v_d2;
                    v_q = v_q2;
                end
            end
        end
        end
    end
end


