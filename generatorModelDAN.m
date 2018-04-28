function [ dPsi_d, dPsi_q, dPsi_r,dPsi_rd,dPsi_rq,electric_torque] =...
    generatorModelDAN(psi,v_d,v_q,e_r,omega_er)
%generatorModelDAN synchronuous generator equations from Merkuriev
%   Detailed explanation goes here
global  omega_er_base torque_base...
        poles_number...
		r_s constant_generator_torque...
        T_rd T_rq T_r N_turb_const omega_m_const;
%% Vector of derivatives
    if constant_generator_torque
        dPsi_d = 0;
        dPsi_q = 0;
        dPsi_r = 0;
        dPsi_rd = 0;
        dPsi_rq = 0;
        electric_torque = -N_turb_const/omega_m_const;
    else
        %% Flux linkage equations
        [psi_d,psi_q,~,~,~] = parsePsi(psi);
        [e_q,e_rq,e_rd,i_q,i_d] = psi_to_E(psi);
        omega_er_pu = omega_er/omega_er_base;
        dPsi_d = omega_er_base.*(-omega_er_pu .* psi_q - i_d .* r_s - v_d);
        dPsi_q = omega_er_base.*(omega_er_pu .* psi_d - i_q .* r_s - v_q);
        dPsi_r = 1/T_r*(e_r-e_q);
        dPsi_rd = -1/T_rd*e_rq;
        dPsi_rq = 1/T_rq*e_rd;
        electric_torque = -(psi_d.*i_q-psi_q.*i_d)*torque_base;
    end
end
