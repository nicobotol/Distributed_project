function [x,u] = sim_dyn(x_bar, u_bar, initial, target, T, K, V_z,dt)
% Dynamic simulation to abtain x_bar and u_bar

n = length(target);

x = zeros(n, T);
u = zeros(n, T);
u(3,:) = V_z;

x(:, 1) = initial;
for m=1:T
    u([1 2 4],m) = u_bar([1 2 4],m)-K{m}([1 2 4],[1 2 4])*(x([1 2 4],m)-x_bar([1 2 4],m));
    x(1, m+1) = x(1,m) + u(1,m)*cos(x(4,m))*dt + u(2,m)*sin(x(4,m))*dt;
    x(2, m+1) = x(2,m) + u(1,m)*sin(x(4,m))*dt + u(2,m)*cos(x(4,m))*dt;
    x(3, m+1) = x(3,m) + u(3,m)*dt;
    x(4, m+1) = x(4,m) + u(4,m)*dt;
end

end