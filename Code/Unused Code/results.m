function [] = results(t, States, Sim_states, Benchmark_states, case_name)

if(~isempty(Benchmark_states)), Benchmark_states.psi_deg = unwrap(Benchmark_states.psi_deg); end

fig1 = figure;
hold on; grid on;
plot3(States(10, :), States(11, :), States(12, :), 'b');
plot3(Sim_states.x, Sim_states.y, Sim_states.z, 'r--');
if(~isempty(Benchmark_states))
    plot3(Benchmark_states.x, Benchmark_states.y, Benchmark_states.z, 'm');
end
view(3);
xlabel("$x_e$ $[ft]$", 'interpreter', 'latex', 'fontSize', 12);
ylabel("$y_e$ $[ft]$", 'interpreter', 'latex', 'fontSize', 12);
zlabel("$z_e$ $[ft]$", 'interpreter', 'latex', 'fontSize', 12);
title("Trajectory");
legend("Matlab", "Simulink", "Benchmark Test", "Location", "southeast");

fig2 = figure;
subplot(3, 1, 1);
hold on; grid on;
plot(t, States(1, :));
plot(Sim_states.time, Sim_states.u, 'r--');
if(~isempty(Benchmark_states)), plot(Benchmark_states.time, Benchmark_states.u, 'm'); end
title("$u$ - $\alpha$ - $\beta$", 'interpreter', 'latex');
ylabel("$u$ $[ft/s]$", 'interpreter', 'latex', 'fontSize', 12);
legend("Matlab", "Simulink", "Benchmark Test", "Location", "southeast");

subplot(3, 1, 2);
hold on; grid on;
plot(t, rad2deg(atan(States(3, :)./States(1, :))));
plot(Sim_states.time, rad2deg(Sim_states.alpha), 'r--');
if(~isempty(Benchmark_states)), plot(Benchmark_states.time, Benchmark_states.alpha_deg, 'm'); end
ylabel("$\alpha$ $[deg]$", 'interpreter', 'latex', 'fontSize', 12);
legend("Matlab", "Simulink", "Benchmark Test", "Location", "southeast");

subplot(3, 1, 3);
hold on; grid on;
plot(t, rad2deg(atan(States(2, :)./States(1, :))));
plot(Sim_states.time, rad2deg(Sim_states.beta), 'r--');
if(~isempty(Benchmark_states)), plot(Benchmark_states.time, Benchmark_states.beta_deg, 'm'); end
xlabel("$t$ [s]", 'interpreter', 'latex', 'fontSize', 12); ylabel("$\beta$ $[deg]$", 'interpreter', 'latex', 'fontSize', 12);
legend("Matlab", "Simulink", "Benchmark Test", "Location", "southeast");

fig3 = figure;
subplot(3, 1, 1);
hold on; grid on;
plot(t, rad2deg(States(4, :)));
plot(Sim_states.time, rad2deg(Sim_states.p), 'r--');
if(~isempty(Benchmark_states)), plot(Benchmark_states.time, rad2deg(Benchmark_states.p), 'm'); end
title("$p$ - $q$ - $r$", 'interpreter', 'latex');
ylabel("$p$ $[deg/s]$", 'interpreter', 'latex', 'fontSize', 12);
legend("Matlab", "Simulink", "Benchmark Test", "Location", "southeast");

subplot(3, 1, 2);
hold on; grid on;
plot(t, rad2deg(States(5, :)));
plot(Sim_states.time, rad2deg(Sim_states.q), 'r--');
if(~isempty(Benchmark_states)), plot(Benchmark_states.time, rad2deg(Benchmark_states.q), 'm'); end
ylabel("$q$ $[deg/s]$", 'interpreter', 'latex', 'fontSize', 12);
legend("Matlab", "Simulink", "Benchmark Test", "Location", "southeast");

subplot(3, 1, 3);
hold on; grid on;
plot(t, rad2deg(States(6, :)));
plot(Sim_states.time, rad2deg(Sim_states.r), 'r--');
if(~isempty(Benchmark_states)), plot(Benchmark_states.time, rad2deg(Benchmark_states.r), 'm'); end
xlabel("$t$ [s]", 'interpreter', 'latex', 'fontSize', 12);
ylabel("$r$ $[deg/s]$", 'interpreter', 'latex', 'fontSize', 12);
legend("Matlab", "Simulink", "Benchmark Test", "Location", "southeast");

fig4 = figure;
subplot(3, 1, 1);
hold on; grid on;
plot(t, rad2deg(States(7, :)));
plot(Sim_states.time, rad2deg(Sim_states.phi), 'r--');
if(~isempty(Benchmark_states)), plot(Benchmark_states.time, Benchmark_states.phi_deg, 'm'); end
title("$\phi$ - $\theta$ - $\psi$", 'interpreter', 'latex');
ylabel("$\phi$ $[deg]$", 'interpreter', 'latex', 'fontSize', 12);
legend("Matlab", "Simulink", "Benchmark Test", "Location", "southeast");

subplot(3, 1, 2);
hold on; grid on;
plot(t, rad2deg(States(8, :)));
plot(Sim_states.time, rad2deg(Sim_states.theta), 'r--');
if(~isempty(Benchmark_states)), plot(Benchmark_states.time, Benchmark_states.theta_deg, 'm'); end
ylabel("$\theta$ $[deg]$", 'interpreter', 'latex', 'fontSize', 12);
legend("Matlab", "Simulink", "Benchmark Test", "Location", "southeast");

subplot(3, 1, 3);
hold on; grid on;
plot(t, rad2deg(States(9, :)));
plot(Sim_states.time, rad2deg(Sim_states.psi), 'r--');
if(~isempty(Benchmark_states)), plot(Benchmark_states.time, Benchmark_states.psi_deg, 'm'); end
xlabel("$t$ [s]", 'interpreter', 'latex', 'fontSize', 12);
ylabel("$\psi$ $[deg]$", 'interpreter', 'latex', 'fontSize', 12);
legend("Matlab", "Simulink", "Benchmark Test", "Location", "southeast");

fig5 = figure;
subplot(3, 1, 1);
hold on; grid on;
plot(t, States(10, :));
plot(Sim_states.time, Sim_states.x, 'r--');
if(~isempty(Benchmark_states)), plot(Benchmark_states.time, Benchmark_states.x, 'm'); end
title("$x_e$ - $y_e$ - $z_e$", 'interpreter', 'latex');
ylabel("$x_e$ $[ft]$", 'interpreter', 'latex', 'fontSize', 12);
legend("Matlab", "Simulink", "Benchmark Test", "Location", "southeast");

subplot(3, 1, 2);
hold on; grid on;
plot(t, States(11, :));
plot(Sim_states.time, Sim_states.y, 'r--');
if(~isempty(Benchmark_states)), plot(Benchmark_states.time, Benchmark_states.y, 'm'); end
ylabel("$y_e$ $[ft]$", 'interpreter', 'latex', 'fontSize', 12);
legend("Matlab", "Simulink", "Benchmark Test", "Location", "southeast");

subplot(3, 1, 3);
hold on; grid on;
plot(t, States(12, :));
plot(Sim_states.time, Sim_states.z, 'r--');
if(~isempty(Benchmark_states)), plot(Benchmark_states.time, Benchmark_states.z, 'm'); end
ylabel("$z_e$ $[ft]$", 'interpreter', 'latex', 'fontSize', 12);
legend("Matlab", "Simulink", "Benchmark Test", "Location", "southeast");
xlabel("$t$ [s]", 'interpreter', 'latex', 'fontSize', 12);

%% Print Figures
saveas(fig1, strcat(case_name, '1'), 'svg');
saveas(fig2, strcat(case_name, '2'), 'svg');
saveas(fig3, strcat(case_name, '3'), 'svg');
saveas(fig4, strcat(case_name, '4'), 'svg');
saveas(fig5, strcat(case_name, '5'), 'svg');

%% Mean Squared Error
if(~isempty(Benchmark_states))
MSE = nan(12, 2);
MSE(1, :) = [meansqr(Benchmark_states.u - States(1,:).'), meansqr(Benchmark_states.u - Sim_states.u.')];
MSE(2, :) = [meansqr(Benchmark_states.v - States(2,:).'), meansqr(Benchmark_states.v - Sim_states.v.')];
MSE(3, :) = [meansqr(Benchmark_states.w - States(3,:).'), meansqr(Benchmark_states.w - Sim_states.w.')];
MSE(4, :) = [meansqr(Benchmark_states.p - States(4,:).'), meansqr(Benchmark_states.p - Sim_states.p.')];
MSE(5, :) = [meansqr(Benchmark_states.q - States(5,:).'), meansqr(Benchmark_states.q - Sim_states.q.')];
MSE(6, :) = [meansqr(Benchmark_states.r - States(6,:).'), meansqr(Benchmark_states.r - Sim_states.r.')];
MSE(7, :) = [meansqr(Benchmark_states.phi_deg - rad2deg(States(7,:)).'), meansqr(Benchmark_states.phi_deg - rad2deg(Sim_states.phi).')];
MSE(8, :) = [meansqr(Benchmark_states.theta_deg - rad2deg(States(8,:)).'), meansqr(Benchmark_states.theta_deg - rad2deg(Sim_states.theta).')];
MSE(9, :) = [meansqr(Benchmark_states.psi_deg - rad2deg(States(9,:)).'), meansqr(Benchmark_states.psi_deg - rad2deg(Sim_states.psi).')];
MSE(10, :) = [meansqr(Benchmark_states.x - States(10,:).'), meansqr(Benchmark_states.x - Sim_states.x.')];
MSE(11, :) = [meansqr(Benchmark_states.y - States(11,:).'), meansqr(Benchmark_states.y - Sim_states.y.')];
MSE(12, :) = [meansqr(Benchmark_states.z - States(12,:).'), meansqr(Benchmark_states.z - Sim_states.z.')];
end

end % endfunction
