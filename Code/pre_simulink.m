clear
close all
clc

%% Problem Setup
aircraft_data_reader;

%% Linearized Matrices
A_longitudinal=[XU,XW,-W0,-g*cos(THETA0), 0, 0;...
        ZU/(1-ZWD),ZW/(1-ZWD),(ZQ+U0)/(1-ZWD),-g*sin(THETA0)/(1-ZWD), 0, 0;...
        MU+MWD*ZU/(1-ZWD),MW+MWD*ZW/(1-ZWD),MQ+MWD*(ZQ+U0)/(1-ZWD),-MWD*g*sin(THETA0)/(1-ZWD), 0, 0;...
        0,0,1,0, 0, 0;
        cos(THETA0), sin(THETA0), 0, -U0*sin(THETA0), 0, 0;
        -sin(THETA0), cos(THETA0), 0, -U0*cos(THETA0), 0, 0];
B_longitudinal=[XDE,XDTH;
        ZDE/(1-ZWD),ZDTH/(1-ZWD);
        MDE+MWD*ZDE/(1-ZWD),MDTH+MWD*ZDTH/(1-ZWD);
        0,0;
        0,0;
        0,0];
C_longitudinal=eye(size(A_longitudinal));
D_longitudinal=zeros(size(B_longitudinal));

% %% Short Period
% A_longitudinal_sp = [A_longitudinal(2,2),A_longitudinal(2,3);A_longitudinal(3,2),A_longitudinal(3,3)];
% B_longitudinal_sp = [B_longitudinal(2,1),B_longitudinal(2,2);B_longitudinal(3,1),B_longitudinal(3,2)];
% C_longitudinal_sp = eye(2);
% D_longitudinal_sp = zeros(2,2);
% 
% %% Long Period
% A_longitudinal_lp = [A_longitudinal(1,1),A_longitudinal(1,4);-ZU/(ZQ+U0) ,0];
% B_longitudinal_lp = [B_longitudinal(1,1),B_longitudinal(1,2);-ZDE/(ZQ+U0),-ZDTH/(ZQ+U0)];
% C_longitudinal_lp = eye(2);
% D_longitudinal_lp = zeros(2,2);

%% Lateral Dynamics
A_lateral = [YV , (W0+YP) , -U0+YR, g*cos(THETA0), 0, 0;
        LV_DASH , LP_DASH , LR_DASH , 0 , 0, 0;
        NV_DASH , NP_DASH , NR_DASH , 0 , 0, 0;
        0 , 1 , tan(THETA0) , 0 , 0, 0;
        0 , 0 , 1/cos(THETA0) ,0 ,0, 0;
        1, 0, 0, 0, U0*cos(THETA0), 0];
B_lateral = [YDA , YDR;
         LDA_DASH , LDR_DASH;
         NDA_DASH , NDR_DASH;
         0 , 0;
         0 , 0;
         0, 0]; % check this, might need fixing
C_lateral=eye(size(A_lateral));
D_lateral=zeros(size(B_lateral));

% %% 1DOF Roll Mode
% A_lateral_1DOF = LP_DASH;
% B_lateral_1DOF = LDA_DASH;
% C_lateral_1DOF = eye(1);
% D_lateral_1DOF = zeros(1,1);
% 
% %% 2DOF Dutch Roll Mode
% A_lateral_2DOF = [YV ,-U0+YR; %- tan(THETA0)*(YP+W0)/U0
%                   NV_DASH , NR_DASH];
% B_lateral_2DOF = [YDA , YDR;  NDA_DASH , NDR_DASH;];
% C_lateral_2DOF = eye(2);
% D_lateral_2DOF = zeros(2,2);
% 
% %% 3DOF Dutch Roll Mode
% A_lateral_3DOF_DR = [YV, 0, -1; LV_DASH, LP_DASH, 0; NV_DASH, 0, NR_DASH];
% B_lateral_3DOF_DR = [YDA, YDR; LDA_DASH, LDR_DASH; NDA_DASH, NDR_DASH];
% C_lateral_3DOF_DR = eye(size(A_lateral_3DOF_DR));
% D_lateral_3DOF_DR = zeros(size(B_lateral_3DOF_DR));
% 
% %% 3DOF Spiral Mode
% A_lateral_3DOF_SP = [LP_DASH, LR_DASH, 0; NP_DASH, NR_DASH, 0; 1, 0, 0];
% B_lateral_3DOF_SP = [LDR_DASH; NDR_DASH; 0];
% C_lateral_3DOF_SP = eye(size(A_lateral_3DOF_SP));
% D_lateral_3DOF_SP = zeros(size(B_lateral_3DOF_SP));

%% Full Linearized Equations
% A_mat = [A_longitudinal, zeros(size(A_longitudinal,1),size(A_lateral,2));
%          zeros(size(A_lateral,1),size(A_longitudinal,2)), A_lateral];
% B_mat = [B_longitudinal, zeros(size(B_longitudinal,1),2);
%          zeros(size(B_lateral,1), 2), B_lateral];
% C_mat = eye(size(A_mat));
% D_mat = zeros(size(B_mat));

%% Frequency Domain
long_SS = ss(A_longitudinal,B_longitudinal,C_longitudinal,D_longitudinal);
long_TF = tf(long_SS);
u_de = long_TF(1, 1);
w_de = long_TF(2,1);
q_de = long_TF(3,1);
theta_de = long_TF(4,1);
u_dth = long_TF(1,2);
w_dth = long_TF(2,2);
q_dth = long_TF(3,2);
theta_dth = long_TF(4,2);

lat_SS = ss(A_lateral,B_lateral,C_lateral,D_lateral);
lat_TF = tf(lat_SS);
v_da = lat_TF(1, 2);
p_da = lat_TF(2,1);
r_da = lat_TF(3,1);
phi_da = lat_TF(4,1);
v_dr = lat_TF(1,2);
p_dr = lat_TF(2,2);
r_dr = lat_TF(3,2);
phi_dr = lat_TF(4,2);

% run('frequency_domain_long.mlx');
% run('frequency_domain_lat.mlx');

%% Longitudinal Autopilot
servo            = tf(10,[1 10]);
integrator       = tf(1,[1 0]);
differentiator   = tf([1 0],1);
engine_timelag   = tf(0.1,[1 0.1]);

%% Elevator from Pitch
OL_theta_thetaCMD = -servo*theta_de;

load("./Autopilot Data/ElevatorFromPitch.mat");
K_ElevFromPitch = tf(C1_ElevFromPitch).num{1}(1);
Ki_ElevFromPitch = tf(C1_ElevFromPitch).num{1}(2);
Kd_ElevFromPitch = tf(C2_ElevFromPitch).num{1}(1);

%% Throttle from Airspeed
OL_u_uCMD = u_dth * engine_timelag * servo;
load("./Autopilot Data/ThrottleFromAirspeed.mat");

%% Pitch from Altitude Altitude
h_theta = -tf(1, [1,0]) * (w_de/theta_de - U0);
OL_h_hCMD = h_theta * CL_theta_thetaCMD;
load("./Autopilot Data/PitchFromAltitude.mat");

%% Yaw Damper
OL_r_rCMD = servo * r_dr;
load("./Autopilot Data/yawDamper.mat");

%% Roll Controller
OL_coordinated_ss = feedback(series(append(1,servo), lat_SS, [1,2], [1,2]), C_yawDamper, 2, 3, 1);
OL_coordinated_tf = tf(OL_coordinated_ss);
OL_phi_phiCMD = minreal(servo * OL_coordinated_tf(4, 1));
load("./Autopilot Data/AileronFromRoll.mat");

%% Simulation Time
tfinal = 20; % final time
dt = 0.01;

%% Bus Creation
Bus_Creator;
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

%% Run Simulink
tic;
out = sim("model.slx");
toc;
fprintf("Finished Solving SIMULINK Code\n");

% save_system("model.slx","model_19a.slx", "ExportToVersion","R2019a");

%% Plot
% fig1 = figure;
% plot(out.time.Data, rad2deg(out.theta_lin.Data), 'LineWidth', 2);
% hold on; grid on;
% % plot(out.time.Data, rad2deg(out.theta.Data), 'LineWidth', 2);
% plot(out.time.Data, rad2deg(out.theta_CMD.Data), 'LineWidth', 2);
% plot(out.time.Data, rad2deg(out.another_theta.Data), 'LineWidth', 2);
% xlabel("t [s]"); ylabel("\theta [deg]");
% legend("\theta_{linear}", "\theta_{CMD}", "\theta_{dE}", 'Location', 'best');
% title("$Pitch Angle$ $\theta$", 'interpreter', 'latex');
% 
% fig2 = figure;
% plot(out.time.Data, out.u_lin.Data, 'LineWidth', 2);
% hold on; grid on;
% % plot(out.time.Data, out.u.Data, 'LineWidth', 2);
% plot(out.time.Data, out.u_CMD.Data, 'LineWidth', 2);
% plot(out.time.Data, out.another_u.Data, 'LineWidth', 2);
% xlabel("t [s]"); ylabel("u [ft/s]");
% title("$u$", 'interpreter', 'latex');
% legend("u_{linear}", "u_{CMD}", "u_{dE}", 'Location', 'best')
% 
% fig3 = figure;
% plot(out.time.Data, rad2deg(out.de.Data), 'LineWidth', 2);
% hold on; grid on;
% xlabel("t [s]"); ylabel("dE [deg]");
% title("$dE$", 'interpreter', 'latex');
% 
% fig4 = figure;
% plot(out.time.Data, out.dth.Data, 'LineWidth', 2);
% hold on; grid on;
% xlabel("t [s]"); ylabel("dTH");
% title("$dTH$", 'interpreter', 'latex');
% 
% fig5 = figure;
% plot(out.time.Data, rad2deg(out.phi_lin.Data), 'LineWidth', 2);
% hold on; grid on;
% plot(out.time.Data, rad2deg(out.phi_CMD.Data), 'LineWidth', 2);
% xlabel("t [s]"); ylabel("$\phi [deg]$", 'interpreter', 'latex', "FontSize", 15);
% legend("\phi_{linear}", "\phi_{CMD}", 'Location', 'best');
% title("$Roll Angle$ $\phi$", 'interpreter', 'latex', "FontSize", 15);
% 
% fig6 = figure;
% plot(out.time.Data, rad2deg(out.psi_lin.Data), 'LineWidth', 2);
% hold on; grid on;
% plot(out.time.Data, rad2deg(out.psi_CMD.Data), 'LineWidth', 2);
% xlabel("t [s]"); ylabel("$\psi$ $[^\circ]$", 'interpreter', 'latex', "FontSize", 15);
% title("$\psi$ $[^\circ]$", 'interpreter', 'latex', "FontSize", 15);
% legend("\psi_{linear}", "\psi_{CMD}", 'Location', 'best')
% 
% fig7 = figure;
% plot(out.time.Data, rad2deg(out.da.Data), 'LineWidth', 2);
% hold on; grid on;
% xlabel("t [s]"); ylabel("$\delta_a$ [deg]", 'interpreter', 'latex', "FontSize", 15);
% title("$\delta_a$", 'interpreter', 'latex', "FontSize", 15);
% 
% fig8 = figure;
% plot(out.time.Data, rad2deg(out.dr.Data), 'LineWidth', 2);
% hold on; grid on;
% xlabel("t [s]"); ylabel("$\delta_r$", 'interpreter', 'latex', "FontSize", 15);
% title("$\delta_r$", 'interpreter', 'latex', "FontSize", 15);
% 
% fig9 = figure;
% plot(out.time.Data, rad2deg(out.beta_lin.Data), 'LineWidth', 2);
% hold on; grid on;
% xlabel("t [s]"); ylabel("$beta$ $[^\circ]$", 'interpreter', 'latex', "FontSize", 15);
% title("$\beta$ $[^\circ]$", 'interpreter', 'latex', "FontSize", 15);

% saveas(fig1, "theta", 'svg')
% saveas(fig2, "u", 'svg')
% saveas(fig3, "dE", 'svg')
% saveas(fig4, "dTH", 'svg')
% saveas(fig5, "phi", 'svg')
% saveas(fig6, "psi", 'svg')
% saveas(fig7, "dA", 'svg')
% saveas(fig8, "dR", 'svg')
% saveas(fig9, "beta", 'svg')
% somefig = figure;
% subplot(6,1,1);
% grid on; hold on;
% plot(out.time.Data, rad2deg(out.phi_lin.Data), 'k', "LineWidth",2);
% xlabel("$t$ [s]", 'Interpreter', 'latex');
% ylabel("$\phi^\circ$", 'Interpreter', 'latex');
% subplot(6,1,2);
% grid on; hold on;
% plot(out.time.Data, rad2deg(out.p_lin.Data), 'k', "LineWidth",2);
% xlabel("$t$ [s]", 'Interpreter', 'latex');
% ylabel("$p$ $[^\circ/s]$", 'Interpreter', 'latex');
% subplot(6,1,3);
% grid on; hold on;
% plot(out.time.Data, rad2deg(out.r.Data), 'k', "LineWidth",2);
% xlabel("$t$ [s]", 'Interpreter', 'latex');
% ylabel("$r$ $[^\circ/s]$", 'Interpreter', 'latex');
% subplot(6,1,4);
% grid on; hold on;
% plot(out.time.Data, rad2deg(out.beta_lin.Data), 'k', "LineWidth",2);
% xlabel("$t$ [s]", 'Interpreter', 'latex');
% ylabel("$\beta$ $[^\circ]$", 'Interpreter', 'latex');
% subplot(6,1,5);
% grid on; hold on;
% plot(out.time.Data, rad2deg(out.dr_lin.Data), 'k', "LineWidth",2);
% xlabel("$t$ [s]", 'Interpreter', 'latex');
% ylabel("$d_R$ $[^\circ]$", 'Interpreter', 'latex');
% subplot(6,1,6);
% grid on; hold on;
% plot(out.time.Data, rad2deg(out.da_lin.Data), 'k', "LineWidth",2);
% xlabel("$t$ [s]", 'Interpreter', 'latex');
% ylabel("$d_A$ $[^\circ]$", 'Interpreter', 'latex');
% saveas(somefig, "yawDamperTest", 'png')

% fig10 = figure;
% subplot(1,2,1);
% grid on; hold on;
% plot(out.time.Data, rad2deg(out.theta_lin.Data), 'LineWidth', 2);
% plot(out.time.Data, rad2deg(out.theta_CMDlin.Data).*ones(size(out.time.Data)), 'LineWidth', 2);
% xlabel("t [s]"); ylabel("\theta [deg]");
% legend("\theta_{linear}", "\theta_{CMD}", 'Location', 'best');
% title("$Pitch Angle$ $\theta$ Linear Autopilot", 'interpreter', 'latex');
% subplot(1,2,2);
% grid on; hold on;
% plot(out.time.Data, rad2deg(out.theta.Data), 'LineWidth', 2);
% plot(out.time.Data, rad2deg(out.theta_CMD.Data).*ones(size(out.time.Data)), 'LineWidth', 2);
% xlabel("t [s]"); ylabel("\theta [deg]");
% legend("\theta", "\theta_{CMD}", 'Location', 'best');
% title("$Pitch Angle$ $\theta$ Nonlinear Autopilot", 'interpreter', 'latex');
% 
% fig11 = figure;
% subplot(1,2,1);
% grid on; hold on;
% plot(out.time.Data, out.altitude_lin.Data, 'LineWidth', 2);
% % plot(out.time.Data, out.h_CMDlin.Data, 'LineWidth', 2);
% xlabel("t [s]"); ylabel("h [ft]");
% % legend("h_{linear}", "h_{CMD}", 'Location', 'best');
% title("$Altitude$ $h$ Linear Autopilot", 'interpreter', 'latex');
% subplot(1,2,2);
% grid on; hold on;
% plot(out.time.Data, out.altitude.Data, 'LineWidth', 2);
% % plot(out.time.Data, out.h_CMD.Data, 'LineWidth', 2);
% xlabel("t [s]"); ylabel("h [ft]");
% % legend("h_{nonlinear}", "h_{CMD}", 'Location', 'best');
% title("$Altitude$ $h$ Nonlinear Autopilot", 'interpreter', 'latex');
% 
% fig12 = figure;
% subplot(1,2,1);
% grid on; hold on;
% plot(out.time.Data, rad2deg(out.phi_lin.Data), 'LineWidth', 2);
% plot(out.time.Data, rad2deg(out.phi_CMDlin.Data), 'LineWidth', 2);
% xlabel("$t$ [s]", 'Interpreter', 'latex', 'FontSize',12); ylabel("$\phi$ [deg]", 'Interpreter', 'latex', 'FontSize',12);
% legend("\phi_{linear}", "\phi_{CMD}", 'Location', 'best');
% title("$Roll Angle$ $\phi$ Linear Autopilot", 'interpreter', 'latex');
% subplot(1,2,2);
% grid on; hold on;
% plot(out.time.Data, rad2deg(out.phi.Data), 'LineWidth', 2);
% plot(out.time.Data, rad2deg(out.phi_CMD.Data), 'LineWidth', 2);
% xlabel("$t$ [s]", 'Interpreter', 'latex', 'FontSize',12); ylabel("$\phi$ [deg]", 'Interpreter', 'latex', 'FontSize',12);
% legend("\phi_{nonlinear}", "\phi_{CMD}", 'Location', 'best');
% title("$Roll Angle$ $\phi$ Nonlinear Autopilot", 'interpreter', 'latex');
% 
% fig13 = figure;
% subplot(1,2,1);
% grid on; hold on;
% plot(out.time.Data, rad2deg(out.psi_lin.Data), 'LineWidth', 2);
% plot(out.time.Data, rad2deg(out.psi_CMDlin.Data), 'LineWidth', 2);
% xlabel("$t$ [s]", 'Interpreter', 'latex', 'FontSize',12); ylabel("$\psi$ [deg]", 'Interpreter', 'latex', 'FontSize',12);
% legend("\psi_{linear}", "\psi_{CMD}", 'Location', 'best');
% title("$Heading Angle$ $\psi$ Linear Autopilot", 'interpreter', 'latex');
% subplot(1,2,2);
% grid on; hold on;
% plot(out.time.Data, rad2deg(out.psi.Data), 'LineWidth', 2);
% plot(out.time.Data, rad2deg(out.psi_CMD.Data), 'LineWidth', 2);
% xlabel("$t$ [s]", 'Interpreter', 'latex', 'FontSize',12); ylabel("$\psi$ [deg]", 'Interpreter', 'latex', 'FontSize',12);
% legend("\psi_{nonlinear}", "\psi_{CMD}", 'Location', 'best');
% title("$Heading Angle$ $\psi$ Nonlinear Autopilot", 'interpreter', 'latex');
% 
% fig14 = figure;
% subplot(1,2,1);
% grid on; hold on;
% plot(out.time.Data, rad2deg(out.beta_lin.Data), 'LineWidth', 2);
% xlabel("$t$ [s]", 'Interpreter', 'latex', 'FontSize',12); ylabel("$\beta$ [deg]", 'Interpreter', 'latex', 'FontSize',12);
% title("$Sideslip Angle$ $\beta$ Linear Autopilot", 'interpreter', 'latex');
% subplot(1,2,2);
% grid on; hold on;
% plot(out.time.Data, rad2deg(out.beta.Data), 'LineWidth', 2);
% xlabel("$t$ [s]", 'Interpreter', 'latex', 'FontSize',12); ylabel("$\beta$ [deg]", 'Interpreter', 'latex', 'FontSize',12);
% title("$Sideslip Angle$ $\beta$ Nonlinear Autopilot", 'interpreter', 'latex');

% fig15 = figure;
% subplot(1,2,1);
% grid on; hold on;
% plot(out.time.Data, rad2deg(out.de_lin.Data), 'LineWidth', 2);
% xlabel("$t$ [s]", 'Interpreter', 'latex', 'FontSize',12); ylabel("$\delta_e$ [deg]", 'Interpreter', 'latex', 'FontSize',12);
% title("$Elevator Response$ $\delta_e$ Linear Autopilot", 'interpreter', 'latex');
% subplot(1,2,2);
% grid on; hold on;
% plot(out.time.Data, rad2deg(out.de.Data), 'LineWidth', 2);
% xlabel("$t$ [s]", 'Interpreter', 'latex', 'FontSize',12); ylabel("$\delta_e$ [deg]", 'Interpreter', 'latex', 'FontSize',12);
% title("$Elevator Action$ $\delta_e$ Nonlinear Autopilot", 'interpreter', 'latex');
% 
% fig16 = figure;
% subplot(1,2,1);
% grid on; hold on;
% plot(out.time.Data, out.dth_lin.Data, 'LineWidth', 2);
% xlabel("$t$ [s]", 'Interpreter', 'latex', 'FontSize',12); ylabel("$\delta_{th}$ [lbf]", 'Interpreter', 'latex', 'FontSize',12);
% title("$Thrust Action$ $\delta_{th}$ Linear Autopilot", 'interpreter', 'latex');
% subplot(1,2,2);
% grid on; hold on;
% plot(out.time.Data, out.dth.Data, 'LineWidth', 2);
% xlabel("$t$ [s]", 'Interpreter', 'latex', 'FontSize',12); ylabel("$\delta_{th}$ [lbf]", 'Interpreter', 'latex', 'FontSize',12);
% title("$Thrust Action$ $\delta_{th}$ Nonlinear Autopilot", 'interpreter', 'latex');

% fig17 = figure;
% subplot(1,2,1);
% grid on; hold on;
% plot(out.time.Data, rad2deg(out.da_lin.Data), 'LineWidth', 2);
% xlabel("$t$ [s]", 'Interpreter', 'latex', 'FontSize',12); ylabel("$\delta_a$ [deg]", 'Interpreter', 'latex', 'FontSize',12);
% title("$Aileron Response$ $\delta_a$ Linear Autopilot", 'interpreter', 'latex');
% subplot(1,2,2);
% grid on; hold on;
% plot(out.time.Data, rad2deg(out.da.Data), 'LineWidth', 2);
% xlabel("$t$ [s]", 'Interpreter', 'latex', 'FontSize',12); ylabel("$\delta_a$ [deg]", 'Interpreter', 'latex', 'FontSize',12);
% title("$Aileron Action$ $\delta_a$ Nonlinear Autopilot", 'interpreter', 'latex');
% 
% fig18 = figure;
% subplot(1,2,1);
% grid on; hold on;
% plot(out.time.Data, rad2deg(out.dr_lin.Data), 'LineWidth', 2);
% xlabel("$t$ [s]", 'Interpreter', 'latex', 'FontSize',12); ylabel("$\delta_r$ [deg]", 'Interpreter', 'latex', 'FontSize',12);
% title("$Rudder Response$ $\delta_r$ Linear Autopilot", 'interpreter', 'latex');
% subplot(1,2,2);
% grid on; hold on;
% plot(out.time.Data, rad2deg(out.dr.Data), 'LineWidth', 2);
% xlabel("$t$ [s]", 'Interpreter', 'latex', 'FontSize',12); ylabel("$\delta_r$ [deg]", 'Interpreter', 'latex', 'FontSize',12);
% title("$Rudder Action$ $\delta_r$ Nonlinear Autopilot", 'interpreter', 'latex');

% fig19 = figure;
% subplot(1,2,1);
% grid on; hold on;
% plot(out.time.Data, rad2deg(out.gamma_lin.Data), 'LineWidth', 2);
% xlabel("$t$ [s]", 'Interpreter', 'latex', 'FontSize',12); ylabel("$\gamma$ [deg]", 'Interpreter', 'latex', 'FontSize',12);
% title("$Flight Path Angle$ $\gamma$ Linear Autopilot", 'interpreter', 'latex');
% subplot(1,2,2);
% grid on; hold on;
% plot(out.time.Data, rad2deg(out.gamma.Data), 'LineWidth', 2);
% xlabel("$t$ [s]", 'Interpreter', 'latex', 'FontSize',12); ylabel("$\gamma$ [deg]", 'Interpreter', 'latex', 'FontSize',12);
% title("$Flight Path Angle$ $\gamma$ Nonlinear Autopilot", 'interpreter', 'latex');

% fig20 = figure;
% subplot(1,2,1);
% grid on; hold on;
% plot(out.time.Data, out.u_lin.Data, 'LineWidth', 2);
% plot(out.time.Data, out.u_CMDlin.Data, 'LineWidth', 2);
% xlabel("t [s]"); ylabel("u [ft/s]");
% legend("u_{linear}", "u_{CMD}", 'Location', 'best');
% title("$Velocity$ $u$ Linear Autopilot", 'interpreter', 'latex');
% subplot(1,2,2);
% grid on; hold on;
% plot(out.time.Data, out.u.Data, 'LineWidth', 2);
% plot(out.time.Data, out.u_CMD.Data, 'LineWidth', 2);
% xlabel("t [s]"); ylabel("u [ft/s]");
% legend("u_{nonlinear}", "u_{CMD}", 'Location', 'best');
% title("$Velocity$ $h$ Nonlinear Autopilot", 'interpreter', 'latex');

% saveas(fig10, "theta", 'svg')
% saveas(fig11, "h", 'svg')
% saveas(fig12, "phi", 'svg')
% saveas(fig13, "psi", 'svg')
% saveas(fig14, "beta", 'svg')
% saveas(fig15, "dE", 'svg')
% saveas(fig16, "dTH", 'svg')
% saveas(fig17, "dA", 'svg')
% saveas(fig18, "dR", 'svg')
% saveas(fig19, "gamma", 'svg')
% saveas(fig20, "u", 'svg')


