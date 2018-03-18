q_is = 100:650;
etas = zeros(size(q_is));
for k=1:length(q_is)
    etas(k) = turbine_eta_m(q_is(k),68);
end
H_turb = 200;
Q_f=@(q_i,H)((q_i.*sqrt(H)*d_runner^2)/1000);
figure(1);
plot(q_is,etas);
figure(2);
plot(q_is,etas*rho*a_g*200.*Q_f(q_is,200)/10^6);