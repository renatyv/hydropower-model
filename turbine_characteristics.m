% clear all;
%     first and last values are extrapolated linearly with
%  e.g. spline(G_10n,G_10q,30)

global eta_Q_u eta_n_u eta_eta_u eta_Qs eta_ns...
    G_nq_q_f n_over_q_required G_required G_nq_q_i Qi_coeff G_n_q_f turbine_eta_m;

G_10q = [130,125,100];
G_10n = [56, 68, 85];

G_14q = [188,175,150, 125];
G_14n = [56, 69, 85.5,95.5];

G_18q = [255,225,200, 175,160];
G_18n = [56, 74, 86.5,95, 100];


G_22q = [312,250,200];
G_22n = [56, 86, 100];

G_26q = [370,300,240];
G_26n = [56,84.5,100];

G_30q = [410,350,280];
G_30n = [56, 84,100];

G_34q = [450,400,320];
G_34n = [56, 82, 100];

G_38q = [480,450,425,350];
G_38n = [56, 78, 86, 100];

G_42q = [520,475, 380];
G_42n = [56, 82.5,100];

G_46q = [550,500,420];
G_46n = [56, 87, 100];

G_54q = [600,575,550,480];
G_54n = [56, 79, 87, 100];

G_58q = [625,600,550,510];
G_58n = [56, 80, 94, 100];

ns = 30:15:100;
G_10q = spline(G_10n,G_10q,ns);
G_14q = spline(G_14n,G_14q,ns);
G_18q = spline(G_18n,G_18q,ns);
G_22q = spline(G_22n,G_22q,ns);
G_26q = spline(G_26n,G_26q,ns);
G_30q = spline(G_30n,G_30q,ns);
G_34q = spline(G_34n,G_34q,ns);
G_38q = spline(G_38n,G_38q,ns);
G_42q = spline(G_42n,G_42q,ns);
G_46q = spline(G_46n,G_46q,ns);
G_54q = spline(G_54n,G_54q,ns);
G_58q = spline(G_58n,G_58q,ns);

G_10v=zeros(1,length(ns))+10;
G_14v=zeros(1,length(ns))+14;
G_18v=zeros(1,length(ns))+18;
G_22v=zeros(1,length(ns))+22;
G_26v=zeros(1,length(ns))+26;
G_30v=zeros(1,length(ns))+30;
G_34v=zeros(1,length(ns))+34;
G_38v=zeros(1,length(ns))+38;
G_42v=zeros(1,length(ns))+42;
G_46v=zeros(1,length(ns))+46;
G_54v=zeros(1,length(ns))+54;
G_58v=zeros(1,length(ns))+58;

% global G_q_i G_n_i G_v;
G_q_i = [G_10q,G_14q,G_18q,G_22q,G_26q,G_30q,G_34q,G_38q,G_42q,G_46q,G_54q,G_58q];
G_n_i = [ns,ns,ns,ns,ns,ns,ns,ns,ns,ns,ns,ns];
G_v = [G_10v,G_14v,G_18v,G_22v,G_26v,G_30v,G_34v,G_38v,G_42v,G_46v,G_54v,G_58v];

% clear -regexp G_\d\dq;
% clear -regexp G_\d\dn;
% clear -regexp G_\d\dv;

%% % % % % % % % % % % % (G,n_i/q_i) -> q_i % % % % % % % % 
Gn_over_q = G_n_i./G_q_i;

% construct uniform grid
G_required = (min(G_v):1:max(G_v));
n_over_q_required = (min(Gn_over_q):0.03:max(Gn_over_q));
[G_u,G_nq_u] = ndgrid(G_required, n_over_q_required);
% interpolate over uniform grid 
% vq = griddata(x,y,v,xq,yq); x,y,v - data; xq, vq - grid
G_nq_q_i = griddata(G_v,Gn_over_q,G_q_i,G_u,G_nq_u);

G_nq_q_f = griddedInterpolant(G_u,G_nq_u,G_nq_q_i);
% G_nq_q_f2 = @(G,n_over_q)(interp2(G_v,Gn_over_q,G_q_i,G,n_over_q));

% assert(G_nq_q_f(12,0.4)-G_nq_q_f2(12,0.4));

%% % % % % % % % % % % % (G,n_i) -> q_i % % % % % % % % 

% construct uniform grid
G_required = (min(G_v):1:max(G_v));
n_required = (min(G_n_i):1:max(G_n_i));
[G_u,G_n_u] = ndgrid(G_required, n_required);
% interpolate over uniform grid 
% vq = griddata(x,y,v,xq,yq); x,y,v - data; xq, vq - grid
G_n_q_i = griddata(G_v,G_n_i,G_q_i,G_u,G_n_u);

G_n_q_f = griddedInterpolant(G_u,G_n_u,G_n_q_i);
% G_nq_q_f2 = @(G,n_over_q)(interp2(G_v,Gn_over_q,G_q_i,G,n_over_q));

% assert(G_nq_q_f(12,0.4)-G_nq_q_f2(12,0.4));

%% % % % % % % % % % % % % n_i,q_i -> G % % % % % % % % 
% % % % % necessary for computation of gates steady state
% Qi_coeff = 1/7;
% G_q_i = G_q_i*Qi_coeff;
% 
% Qs = (min(G_q_i):2:max(G_q_i));
% ns = (min(G_n_i):2:max(G_v));
% [n_u,q_u] = meshgrid(ns, Qs);
% G_u = griddata(G_n_i,G_q_i,G_v,n_u,q_u);
% % 
% G_u = G_u'; % G values
% n_u = n_u';
% q_u=q_u';
% %     figure(1);
% %     mesh(n_u,q_u,G_u);
% %     xlabel('n_i, G');
% %     ylabel('q_i/10');
% %     zlabel('G');
% q_i_n_i_to_G = griddedInterpolant(n_u,q_u,G_u,'linear');
% niqi_to_G = @(n_i,q_i)(q_i_n_i_to_G(n_i,q_i*Qi_coeff));


%% q_i, n_i->eta
nu_922q=[418,424];
nu_922n=[66.2,66.7];
nu_922v=zeros(1,length(nu_922q))+0.922;

nu_92q=[400, 425,435, 445, 430,400, 390];
nu_92n=[64.5, 65, 66,  67.5,68, 67.5,66];
nu_92v=zeros(1,length(nu_92q))+0.92;

nu_91q=[380,400,425,450,470,470,450, 425,400, 387.5,380];
nu_91n=[64, 63, 63, 64, 67, 68, 71.5,71, 69.2,68, 66];
nu_91v=zeros(1,length(nu_91q))+0.91;

nu_90q=[400, 425, 450, 475, 500, 512,520,512,500, 475, 450, 425,390,350,  337,350];
nu_90n=[58.3,58.5,59.3,60.7,63.5,67, 70, 72, 73.2,75.5,75.8,74, 71.3,68.5,65, 60.5];
nu_90v=zeros(1,length(nu_90q))+0.9;

nu_88q=[320,300,290,300,325,350, 375, 400,425, 450,475, 500, 525,550,560,550, 525,460];
nu_88n=[56, 60, 62, 65, 71, 73.5,75.5,77, 78.2,79, 79.5,78.5,77, 74, 70, 65.5,61, 56];
nu_88v=zeros(1,length(nu_88q))+0.88;

nu_84q=[290,239,230,233,250, 275,300, 350,400,450,475, 500, 525, 575, 600,618,618,600,575,550];
nu_84n=[55, 58, 63, 65, 69.5,73, 75.5,79, 81, 83, 83.5,83.5,82.5,78.7,76, 72, 67, 63, 59, 56];
nu_84v=zeros(1,length(nu_84q))+0.84;

nu_80q=[180,175,175,200,225,250, 275, 300, 325, 350, 375,400,425, 450,475, 500, 512,525, 550,575, 630,650,660];
nu_80n=[56, 59,  64, 70, 74,76.5,78.2,79.5,81.5,81.7,83, 84, 85.3,86, 87.1,87.2,87, 86.5,85,83.7,77, 74, 70];
nu_80v=zeros(1,length(nu_80q))+0.8;

nu_70q=[118,120,125,150, 190,275,450,475, 500,525, 550,600, 650,700];
nu_70n=[57, 64, 67, 74.2,80, 85.5,92,92.7,93, 92.7,91.7,87, 83.5,70];
nu_70v=zeros(1,length(nu_70q))+0.7;

nu_60q=[180,100, 125,150,175,225,275,350,400,450,500,525];
nu_60n=[45, 68.5,75.5,80,84, 87, 90, 94, 94.5,96.5,97,96.7];
nu_60v=zeros(1,length(nu_60q))+0.6;

nu_50q=[100, 125, 150,175,200,225,250, 275, 325, 350, 400,440];
nu_50n=[75.5,80.5,84.5,87,90, 92, 93.5,94.5,96.5,97.5,99, 100];
nu_50v=zeros(1,length(nu_50q))+0.5;

nu_40q=[100,125,175,200,250,300];
nu_40n=[80,84.5,91.7,94,97.3,98.5];
nu_40v=zeros(1,length(nu_40q))+0.4;

nu_30q=[100,125,150,175,200,230];
nu_30n=[83, 88, 92, 95,97.2,100];
nu_30v=zeros(1,length(nu_30q))+0.3;

nu_10q=[50,100, 125, 150,175];
nu_10n=[50,88.5,93.5,97,100];
nu_10v=zeros(1,length(nu_10q))+0.1;

nu_0q=[0,0,  30,30,30,30,35,40,100, 125, 150,158,700,700];
nu_0n=[0,100,30,50,60,70,75,80,90,  96,  99,100, 0,  100];
nu_0v=zeros(1,length(nu_0q));

% global nu_q_i nu_n_i eta_i;
nu_q_i = [nu_922q,nu_92q,nu_91q,nu_90q,nu_88q,nu_84q,nu_80q,nu_70q,nu_60q,nu_50q,nu_40q,nu_30q,nu_10q,nu_0q]; 
nu_n_i = [nu_922n,nu_92n,nu_91n,nu_90n,nu_88n,nu_84n,nu_80n,nu_70n,nu_60n,nu_50n,nu_40n,nu_30n,nu_10n,nu_0n];
eta_i = [nu_922v,nu_92v,nu_91v,nu_90v,nu_88v,nu_84v,nu_80v,nu_70v,nu_60v,nu_50v,nu_40v,nu_30v,nu_10v,nu_0v];

% clear -regexp nu_\d\dq;
% clear -regexp nu_\d\dn;
% clear -regexp nu_\d\dv;
% clear nu_0q nu_0v nu_0n;

%     makes delanay triangulation adequate, because Q and n are of the same
%     scale
Qi_coeff = 1/7;
nu_q_i = nu_q_i*Qi_coeff;
%     preparing uniform grid for resampling
%   resampling data to uniform grid
eta_Qs = (0:7:700)*Qi_coeff;
eta_ns = 0:1:100;
[eta_Q_u,eta_n_u] = ndgrid(eta_Qs, eta_ns);
%     resampling the universal characteristic
% q_i,n_i -> eta
eta_eta_u = griddata(nu_q_i,nu_n_i,eta_i,eta_Q_u,eta_n_u);
%     preparing data for griddedInterpolant, which is faster than interp2

% eta_Q_u = eta_Q_u';
% eta_n_u = eta_n_u';
% eta_eta_u = eta_eta_u';

q_in_i_to_eta = griddedInterpolant(eta_Q_u,eta_n_u,eta_eta_u);

% % % % % necessary for computation of turbine steady state
turbine_eta_m = @(q_i_input,n_i_input)(q_in_i_to_eta(q_i_input*Qi_coeff,n_i_input));