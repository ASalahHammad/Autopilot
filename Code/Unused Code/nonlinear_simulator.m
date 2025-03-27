% clear; clc; close all;

%% Problem Setup
if(~exist("tf", "var"))
    tfinal = 200; % final time
    % n_iter = 20001;
    dt = 0.01; % time step
end
t = (0:dt:tfinal).'; % Time Vector

if(~exist("controls", "var"))
    controls = [5*pi/180; 0; 0; 0];
end
dA = controls(1);
dE = controls(2);
dTH = controls(3);
dR = controls(4);

aircraft_data_reader;
% Bus Creation
States_Bus_Creator;
Controls_Bus_Creator;
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

%% Simulink
tic;
Sim_states = sim("model.slx");
toc;
fprintf("Finished Solving SIMULINK Code\n");

%% Our RK4 Solver
tic;
[t, States] = RK4(@(t, states, wdot) Fdot(t, states, wdot, s0, controls, States_Matrix, Controls_Matrix, m, g, I, invI, mg0), t, s0);
toc;
fprintf("Finished Solving MATLAB Code\n");

% %% Benchmark Test
% load('Benchmark_B747_FC5.mat');

%% Results
Sim_states.u=Sim_states.u.Data.';Sim_states.v=Sim_states.v.Data.';Sim_states.w=Sim_states.w.Data.';
Sim_states.alpha=Sim_states.alpha.Data.';Sim_states.beta=Sim_states.beta.Data.';
Sim_states.p=Sim_states.p.Data.';Sim_states.q=Sim_states.q.Data.';Sim_states.r=Sim_states.r.Data.';
Sim_states.phi=Sim_states.phi.Data.';Sim_states.theta=Sim_states.theta.Data.';Sim_states.psi=Sim_states.psi.Data.';
Sim_states.x=Sim_states.x.Data.';Sim_states.y=Sim_states.y.Data.';Sim_states.z=Sim_states.z.Data.';

Sim_states.time = Sim_states.time.Data;
