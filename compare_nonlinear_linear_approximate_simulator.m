clear;
clc;
close all;

% control inputs
controls_vec = [0, 5*pi/180, -5*pi/180, 0, 0, 0, 0, 0, 0;
                0, 0, 0, 5*pi/180, -5*pi/180, 0, 0, 0, 0;
                0, 0, 0, 0, 0, 1000, 10000, 0, 0;
                0, 0, 0, 0, 0, 0, 0, 5*pi/180, -5*pi/180];

case_names = ["no_input", "pos_5deg_daileron", "neg_5deg_daileron", "pos_5deg_delevator", "neg_5deg_delevator", "1000_dthrottle", "10000_dthrottle", "pos_5deg_drudder", "neg_5deg_drudder"];
for i = 1:length(case_names)
    dt = 0.01;
    if(i==1)
        tf = 2000;
    else
        tf = 200;
    end
    case_name = case_names(i);
    controls = controls_vec(:, i);
    dA = controls(1);
    dE = controls(2);
    dTH = controls(3);
    dR = controls(4);
    main;
    Benchmark_states = eval(strcat("Benchmark_B747_FC5_", case_name));
    MSE = results(t, States, Sim_states, Benchmark_states, case_name);
    disp(case_name);
    disp(max(MSE));
end
