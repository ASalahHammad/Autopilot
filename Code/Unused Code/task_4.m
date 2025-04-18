clear
close all
clc

%% Problem Setup
tfinal = 5; % final time
% n_iter = 200001;
% dt = tf/(n_iter-1); % time step
dt = 0.01;
t = (0:dt:tfinal).'; % Time Vector

% controls = [0; 0; 0; 0];
dE = 0 *pi/180;
dTH = 0;
dA = 25 *pi/180;
dR = 0 *pi/180;

aircraft_data_reader;
% Bus Creation
States_Bus_Creator;
initial_states_bus.u = s0(1);
initial_states_bus.v = s0(2);
initial_states_bus.w = s0(3);
initial_states_bus.p = s0(4);
initial_states_bus.q = s0(5);
initial_states_bus.r = s0(6);
initial_states_bus.phi = s0(7);
initial_states_bus.theta = s0(8);
initial_states_bus.psi = s0(9);
initial_states_bus.x = s0(10);
initial_states_bus.y = s0(11);
initial_states_bus.z = s0(12);
initial_states_bus.alpha = s0(8);
initial_states_bus.beta = 0;
initial_states_bus.wdot = 0;

%% Linearized Matrices
A_longitudinal=[XU,XW,-W0,-g*cos(theta0);...
        ZU/(1-ZWD),ZW/(1-ZWD),(ZQ+U0)/(1-ZWD),-g*sin(theta0)/(1-ZWD);...
        MU+MWD*ZU/(1-ZWD),MW+MWD*ZW/(1-ZWD),MQ+MWD*(ZQ+U0)/(1-ZWD),-MWD*g*sin(theta0)/(1-ZWD);...
        0,0,1,0];
B_longitudinal=[XDE,XDTH ;...
        ZDE/(1-ZWD),ZDTH/(1-ZWD);...
        MDE+MWD*ZDE/(1-ZWD),MDTH+MWD*ZDTH/(1-ZWD);...
        0,0];
C_longitudinal=eye(4);
D_longitudinal=zeros(4,2);

%% Short Period
A_longitudinal_sp = [A_longitudinal(2,2),A_longitudinal(2,3);A_longitudinal(3,2),A_longitudinal(3,3)];
B_longitudinal_sp = [B_longitudinal(2,1),B_longitudinal(2,2);B_longitudinal(3,1),B_longitudinal(3,2)];
C_longitudinal_sp = eye(2);
D_longitudinal_sp = zeros(2,2);

%% Long Period
A_longitudinal_lp = [A_longitudinal(1,1),A_longitudinal(1,4);-ZU/(ZQ+U0) ,0];
B_longitudinal_lp = [B_longitudinal(1,1),B_longitudinal(1,2);-ZDE/(ZQ+U0),-ZDTH/(ZQ+U0)];
C_longitudinal_lp = eye(2);
D_longitudinal_lp = zeros(2,2);

%% Lateral Dynamics
A_lateral = [YV , (W0+YP) , -U0+YR, g*cos(theta0), 0;...
        LV_DASH , LP_DASH , LR_DASH , 0 , 0;...
        NV_DASH , NP_DASH , NR_DASH , 0 , 0;...
        0 , 1 , tan(theta0) , 0 , 0;...
        0 , 0 , 1/cos(theta0) ,0 ,0];
B_lateral = [YDA , YDR;...
         LDA_DASH , LDR_DASH;...
         NDA_DASH , NDR_DASH;...
         0 , 0;...
         0 , 0]; % check this, might need fixing
C_lateral=eye(5);
D_lateral=zeros(5,2);

%% 1DOF Roll Mode
A_lateral_1DOF = LP_DASH;
B_lateral_1DOF = LDA_DASH;
C_lateral_1DOF = eye(1);
D_lateral_1DOF = zeros(1,1);

%% 2DOF Dutch Roll Mode
A_lateral_2DOF = [YV ,-U0+YR; %- tan(theta0)*(YP+W0)/U0
                  NV_DASH , NR_DASH];
B_lateral_2DOF = [YDA , YDR;  NDA_DASH , NDR_DASH;];
C_lateral_2DOF = eye(2);
D_lateral_2DOF = zeros(2,2);

%% 3DOF Dutch Roll Mode
A_lateral_3DOF_DR = [YV, 0, -1; LV_DASH, LP_DASH, 0; NV_DASH, 0, NR_DASH];
B_lateral_3DOF_DR = [YDA, YDR; LDA_DASH, LDR_DASH; NDA_DASH, NDR_DASH];
C_lateral_3DOF_DR = eye(size(A_lateral_3DOF_DR));
D_lateral_3DOF_DR = zeros(size(B_lateral_3DOF_DR));

%% 3DOF Spiral Mode
A_lateral_3DOF_SP = [LP_DASH, LR_DASH, 0; NP_DASH, NR_DASH, 0; 1, 0, 0];
B_lateral_3DOF_SP = [LDR_DASH; NDR_DASH; 0];
C_lateral_3DOF_SP = eye(size(A_lateral_3DOF_SP));
D_lateral_3DOF_SP = zeros(size(B_lateral_3DOF_SP));

%% Full Linearized Equations
A_mat = [A_longitudinal, zeros(size(A_longitudinal,1),size(A_lateral,2)), zeros(size(A_longitudinal,1), 3);
         zeros(size(A_lateral,1),size(A_longitudinal,2)), A_lateral, zeros(size(A_lateral,1), 3);
         cos(theta0), sin(theta0), 0, -U0*sin(theta0), zeros(1, size(A_lateral,2)+3);
         zeros(1,size(A_longitudinal,2)), 1, 0, 0, 0, U0*cos(theta0), zeros(1,3);
         -sin(theta0), cos(theta0), 0, -U0*cos(theta0), zeros(1, size(A_lateral,2)+3)];
B_mat = [B_longitudinal, zeros(size(B_longitudinal,1),2);
         zeros(size(B_lateral,1), 2), B_lateral;
         zeros(3,4)];
C_mat = eye(size(A_mat));
D_mat = zeros(size(B_mat));

run('frequency_domain_long.mlx');
% run('frequency_domain_lat.mlx');

%% Longitudinal Autopilot
%% given trasnfer functions
servo            = tf(10,[1 10]);
integrator       = tf(1,[1 0]);
differentiator   = tf([1 0],1);
engine_timelag   = tf(0.1,[1 0.1]);

%pitch controller   
theta_de_extended_plant = -1 * servo *theta_de;
u_dth_extended_plant = servo * engine_timelag *u_dth

load('Long_Pitch_controller.mat');
load('Long_vel_controller.mat');
load('Long_Alt_controller.mat');

h_theta       = minreal(-integrator*(w_de/theta_de-U0));
OL_h_thetacom = tf(minreal(CL_theta_thetacom*h_theta));

K = PI_Pitch.K;
K_i = -K * PI_Pitch.Z{1};
K_d_theta = D_Pitch.K;

%% Simulink
tic;
out = sim("model.slx");
toc;
fprintf("Finished Solving SIMULINK Code\n");

%% Plot
title_name = strcat("Controls: dE=",num2str(dE*180/pi), " [deg], dTH=",num2str(dTH), ", dA=",num2str(dA*180/pi), " [deg], dR=",num2str(dR*180/pi), " [deg]");

fig1 = figure;
plot(out.time.Data, rad2deg(out.alpha.Data), 'LineWidth', 2);
hold on;
plot(out.time.Data, rad2deg(out.alpha_lin.Data), 'LineWidth', 2);
plot(out.time.Data, rad2deg(atan(out.w_short_period.Data/U0)), 'LineWidth', 2);
xlabel("t [s]"); ylabel("w [ft/s]");
legend("Nonlinear", "Linearized", "Short Period")
title(title_name, 'interpreter', 'latex');

fig2 = figure;
plot(out.time.Data, rad2deg(out.q.Data), 'LineWidth', 2)
hold on;
plot(out.time.Data, rad2deg(out.q_lin.Data), 'LineWidth', 2)
plot(out.time.Data, rad2deg(out.q_short_period.Data), '--', 'LineWidth', 2)
xlabel("t [s]"); ylabel("q [deg/s]");
legend("Nonlinear", "Linearized", "Short Period")
title(title_name, 'interpreter', 'latex');

fig3 = figure;
plot(out.time.Data, out.u.Data, 'LineWidth', 2)
hold on
plot(out.time.Data, out.u_lin.Data, 'LineWidth', 2)
plot(out.time.Data, out.u_long_period.Data, 'LineWidth', 2)
xlabel("t [s]"); ylabel("u [ft/s]");
legend("Nonlinear", "Linearized", "Long Period")
title(title_name, 'interpreter', 'latex');

fig4 = figure;
plot(out.time.Data, rad2deg(out.theta.Data), 'LineWidth', 2)
hold on
plot(out.time.Data, rad2deg(out.theta_lin.Data), 'LineWidth', 2)
plot(out.time.Data, rad2deg(out.theta_long_period.Data), 'LineWidth', 2)
xlabel("t [s]"); ylabel("theta [deg]");
legend("Nonlinear", "Linearized", "Long Period")
title(title_name, 'interpreter', 'latex');

% saveas(fig1, strcat(title_name, '__1'), 'svg');
% saveas(fig2, strcat(title_name, '__2'), 'svg');
% saveas(fig3, strcat(title_name, '__3'), 'svg');
% saveas(fig4, strcat(title_name, '__4'), 'svg');

% fig5 = figure; hold on;
% plot(out.time.Data, out.v.Data, 'LineWidth', 2);
% plot(out.time.Data, out.v_lin.Data, 'LineWidth', 2);
% plot(out.time.Data, out.v_2DOF.Data, 'LineWidth', 2);
% plot(out.time.Data, out.v_3DOF_DR.Data, 'LineWidth', 2);
% xlabel("time [sec]"); ylabel("v [ft/s]");
% legend("Nonlinear", "Linearized", "2DOF", "3DOF\_DR");
% title(title_name, 'interpreter', 'latex');
% 
% fig6 = figure; hold on;
% plot(out.time.Data, rad2deg(out.p.Data), 'LineWidth', 2);
% plot(out.time.Data, rad2deg(out.p_lin.Data), 'LineWidth', 2);
% plot(out.time.Data, rad2deg(out.p_1DOF.Data), 'LineWidth', 2);
% plot(out.time.Data, rad2deg(out.p_3DOF_DR.Data), 'LineWidth', 2);
% plot(out.time.Data, rad2deg(out.p_3DOF_SP.Data), 'LineWidth', 2);
% xlabel("time [sec]"); ylabel("p [deg/s]")
% legend("Nonlinear", "Linearized", "1DOF", "3DOF\_DR", "3DOF\_SP");
% title(title_name, 'interpreter', 'latex');
% 
% fig7 = figure; hold on;
% plot(out.time.Data, rad2deg(out.r.Data), 'LineWidth', 2);
% plot(out.time.Data, rad2deg(out.r_lin.Data), 'LineWidth', 2);
% plot(out.time.Data, rad2deg(out.r_2DOF.Data), 'LineWidth', 2);
% plot(out.time.Data, rad2deg(out.r_3DOF_DR.Data), 'LineWidth', 2);
% plot(out.time.Data, rad2deg(out.r_3DOF_SP.Data), 'LineWidth', 2);
% xlabel("time [sec]"); ylabel("r [deg/s]");
% legend("Nonlinear", "Linearized", "2DOF", "3DOF\_DR", "3DOF\_SP");
% title(title_name, 'interpreter', 'latex');
% 
% fig8 = figure; hold on;
% plot(out.time.Data, rad2deg(out.phi.Data), 'LineWidth', 2);
% plot(out.time.Data, rad2deg(out.phi_lin.Data), 'LineWidth', 2);
% plot(out.time.Data, rad2deg(out.phi_3DOF_SP.Data), 'LineWidth', 2);
% xlabel("time [sec]"); ylabel("\phi [deg]");
% legend("Nonlinear", "Linearized", "3DOF\_SP");
% title(title_name, 'interpreter', 'latex');
% 
% saveas(fig5, strcat(title_name, '__5'), 'svg');
% saveas(fig6, strcat(title_name, '__6'), 'svg');
% saveas(fig7, strcat(title_name, '__7'), 'svg');
% saveas(fig8, strcat(title_name, '__8'), 'svg');
