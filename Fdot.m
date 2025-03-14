function [states_dot] = Fdot(t, states, wdot, states_0, controls, States_Matrix, Controls_Matrix, mass, gravity, I, invI, mg0)

states_dot = nan(size(states));

u = states(1);
v = states(2);
w = states(3);
p = states(4);
q = states(5);
r = states(6);
phi = states(7);
theta = states(8);
psi = states(9);
x = states(10);
y = states(11);
z = states(12);

J = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
     0,        cos(phi),               -sin(phi);
     0, sin(phi)/cos(theta), cos(phi)/cos(theta)];

R = eul2rotm([psi, theta, phi], "ZYX");

d_states = states - states_0; % perturbation in states around equilibrium point
% Compute Total Forces
[F, M] = forces_moments(t, d_states, controls, States_Matrix, Controls_Matrix, wdot);
F = F*mass + R.' * [0;0;mass*gravity] + mg0;
M = M.*[I(1,1);I(2,2);I(3,3)] + 0;

% state space model
states_dot(1:3) = 1/mass*F - cross([p; q; r], [u; v; w]); % u,v,w
states_dot(4:6) = invI*(M - cross([p; q; r], I*[p; q; r])); % p,q,r
states_dot(7:9) = J * [p; q; r]; % phi,theta,psi
states_dot(10:12) = R * [u; v; w]; % x,y,z

end
