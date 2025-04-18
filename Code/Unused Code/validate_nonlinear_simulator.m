clear;
clc;
close all;

% control inputs
% controls_vec = [0, 5*pi/180, -5*pi/180, 0, 0, 0, 0, 0, 0;
%                 0, 0, 0, 5*pi/180, -5*pi/180, 0, 0, 0, 0;
%                 0, 0, 0, 0, 0, 1000, 10000, 0, 0;
%                 0, 0, 0, 0, 0, 0, 0, 5*pi/180, -5*pi/180];

% case_names = ["no_input", "pos_5deg_daileron", "neg_5deg_daileron", "pos_5deg_delevator", "neg_5deg_delevator", "1000_dthrottle", "10000_dthrottle", "pos_5deg_drudder", "neg_5deg_drudder"];
controls_vec = [5*pi/180; 0; 0; 0];
case_names = ["pos_5deg_daileron"];

for i = 1:length(case_names)
    case_name = case_names(i);
    disp(case_name);
    dt = 0.01;
    if(i==1)
        tfinal = 2000;
    else
        tfinal = 200;
    end
    controls = controls_vec(:, i);
    dA = controls(1);
    dE = controls(2);
    dTH = controls(3);
    dR = controls(4);
    nonlinear_simulator;
    % Benchmark_states = eval(strcat("Benchmark_B747_FC5_", case_name));
    % MSE = results(t, States, Sim_states, Benchmark_states, case_name);
    results(t, States, Sim_states, [], strcat("Jetstar_", case_name));
    % disp(max(MSE));
end
