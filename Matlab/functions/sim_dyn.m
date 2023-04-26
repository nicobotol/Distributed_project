function [x,u] = sim_dyn(x_bar, u_bar, n, T, initial, K, dt)
% Dynamic simulation to abtain x_bar and u_bar

x = zeros(n-1, T);
u = zeros(n-1, T);

x(:, 1) = initial;
for m=1:T
    u(:,m) = u_bar(:,m)+K{m}*(x(:,m)-x_bar(:,m));
    x(1, m+1) = x(1,m) + u(1,m)*cos(x(3,m))*dt - u(2,m)*sin(x(3,m))*dt;
    x(2, m+1) = x(2,m) + u(1,m)*sin(x(3,m))*dt + u(2,m)*cos(x(3,m))*dt;
    x(3, m+1) = x(3,m) + u(3,m)*dt;
end

end