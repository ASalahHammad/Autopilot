function [t, states] = RK4(f_dot, t, states_0)

states = nan(length(states_0), length(t));
states(:, 1) = states_0;
wdot = 0;

for n = 1:length(t)-1
    h = t(n+1) - t(n);

    K1 = f_dot(t(n), states(:, n), wdot);
    wdot = K1(3);
    K2 = f_dot(t(n)+h/2, states(:, n)+0.5*K1*h, wdot);
    wdot = K2(3)*.5;
    K3 = f_dot(t(n)+h/2, states(:, n)+0.5*K2*h, wdot);
    wdot = K3(3)*.5;
    K4 = f_dot(t(n)+h, states(:, n)+K3*h, wdot);
    wdot = K4(3);
    states(:, n+1) = states(:, n) + h/6*(K1 + 2*K2 + 2*K3 + K4);

end

end % endfunction
