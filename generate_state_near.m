function [near_state] = generate_state_near(gov_model,gen_model,turb_model,load_model,state,max_distance)
%generate_state_near generates valid steady state near the state within radius epsilon
H= -0.5;

while H<100 || H>300 %|| P_active<0 || P_active>700*10^6
    dState = (rand(size(state))-0.5)*max_distance;
    near_state = state+dState;
%%     saturate the servos
    near_state(5) = sat(near_state(5),0,1);
    near_state(3) = sat(near_state(3),0,1);
    G = near_state(3)*turb_model.G_base;
    Q = near_state(2)*turb_model.Q_base;
    H=(Q/(turb_model.gate_flow_coeff*G))^2;
end

end

