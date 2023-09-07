function [x_plus] = unicycle_dynamics(x, u, nu, dt)
%% This function propagates the unicycle dynamic
% u = [v, omega, v_z]';
% x = [x, y, z, theta]';
% nu = [nu_vx, nu_vy, nu_vz, Vz, nu_theta]';
% dt = time step

x_plus = x + [u(1)*cos(x(4))*dt + nu(1);
              u(1)*sin(x(4))*dt + nu(2);
              u(3)*dt + nu(3) + nu(4)*dt;
              u(2)*dt + nu(5)];

end