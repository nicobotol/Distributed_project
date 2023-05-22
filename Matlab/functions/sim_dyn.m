function [x,u] = sim_dyn(x_bar, u_bar, n, T, initial, K, dt, V)
% Dynamic simulation to abtain x_bar and u_bar

x = zeros(n-1, T);
u = zeros(1, T);
nu = zeros(n-1, T);

x(:, 1) = initial;
for m=1:T
    nu(:,m) = 0*randn(3,1);
    u(:,m) = u_bar(:,m)+K{m}*(x(:,m)-x_bar(:,m));
    x(1, m+1) = x(1,m) - V*sin(x(3,m)) + nu(1,m);
    x(2, m+1) = x(2,m) + V*cos(x(3,m)) + nu(2,m);
    x(3, m+1) = x(3,m) + u(m)*dt + nu(3,m);
end

end