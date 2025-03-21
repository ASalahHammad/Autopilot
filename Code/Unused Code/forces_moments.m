function [F, M] = forces_moments(t, d_states, controls, states_matrix, controls_matrix, wdot)

%% This is force per unit mass and moment per [I(xx/yy/zz)]
FM = states_matrix*[d_states(1:6); wdot] + controls_matrix*controls;
F = FM(1:3);
M = FM(4:6);

end
