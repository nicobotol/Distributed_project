clc
close all
clear

set(0,'DefaultFigureWindowStyle','docked');

rng(3);

I = 1; % inertia [kg*m^2]
dt = 0.01; % time steep [s]
sim_t = 10; % simulation time [s]
T = sim_t/dt; % number of iterations [-]
t_vect = dt:dt:sim_t; % [s]
k_p = 1000; % proportional gain for the controller
k_i = 0;
marker_size = 10;
line_width = 2;


% Target point
target = [0 0 0]'; % [x y z] [m]

Gh = eye(4, 4); % linearized disturbance matrix

x = zeros(4, T);        % state
u = zeros(3, T);        % input
x(:, 1) = [30 30 50 pi];   % initialize the state
V_x = sqrt(sum((x(1, 1:3) - target').^2))/sim_t;                % longitudinal speed [m/s]
V_z = 5;              % vertical speed [m/s]

% Covariance amtrix for the uncertainty
R_GPS = rand(4, 4) - 0.5; % 4 state
R_GPS = R_GPS*R_GPS';

% Input covariance matrix
Qi = 0.1*(rand(4, 4) - 0.5); % 3 inputs
Qi = Qi*Qi';
nu = zeros(4, 1);

% State estimate
x_est = zeros(4, T);
x_est(:, 1) = x(:, 1);
P_est = zeros(4, 4);
e_theta = zeros(T, 1);


for t=1:T-1
  alpha = atan2(x_est(2, t) - target(2), x_est(1, t) - target(1));
  e_theta(t + 1) = pi + alpha - x_est(4, t); % angular erro for the controller
  m = k_p*e_theta(t + 1) + k_i*((e_theta(t) + e_theta(t + 1))/2*dt); % torque proportional to angula error
  u(:, t) = [V_x V_z m]';             % input
  nu(:) = mvnrnd([0; 0; 0; 0], Qi)';  % noise on the input and on the model

  % Update the state
  x(1, t+1) = x(1, t) + u(1, t)*cos(x(4, t))*dt + nu(1);
  x(2, t+1) = x(2, t) + u(1, t)*sin(x(4, t))*dt + nu(2);
  x(3, t+1) = x(3, t) - u(2, t)*dt + nu(3);
  x(4, t+1) = x(4, t) + u(3, t)*dt^2/(2*I) + nu(4);
  
  % Prediction
  x_est(1, t+1) = x_est(1, t) + u(1, t)*cos(x_est(4, t))*dt;
  x_est(2, t+1) = x_est(2, t) + u(1, t)*sin(x_est(4, t))*dt;
  x_est(3, t+1) = x_est(3, t) - u(2, t)*dt;
  x_est(4, t+1) = x_est(4, t) + u(3, t)*dt^2/(2*I);

  % Linearized Ah(k)
  Ah = eye(4);
  Ah(1, 4) = -u(1, t)*sin(x_est(4, t))*dt;
  Ah(2, 4) = u(1, t)*cos(x_est(4, t))*dt;
  
  P_est = Ah*P_est*Ah' + Gh*Qi*Gh'; 

  % Measurement update using the GPS
  z_GPS = x(:, t + 1) + mvnrnd([0; 0; 0; 0], R_GPS)'; % measurement
  Innovation = z_GPS - x_est(:, t + 1);
  H_GPS = eye(4);  % linearized model of the GPSs
  % update the kalaman estimate
  S_Inno = H_GPS*P_est*H_GPS' + R_GPS;
  W = P_est*H_GPS'*inv(S_Inno); % kalman gain
  x_est(:, t + 1) = x_est(:, t + 1) + W*Innovation; % update stte estimate
  P_est = (eye(4) - W*H_GPS)*P_est; % update covariance matrix

  % Check if we have touch the ground
  if x(3, t) < 0
    break
  end
end

%% Plots
drawArrow = @(x,y) quiver( x(1),y(1),x(2)-x(1),y(2)-y(1),0 );
drawArrow3 = @(x,y, z) quiver( x(1),y(1),x(2)-x(1),y(2)-y(1),z);
arrow_mag = 5;
figure()
plot(x(1,:), x(2, :))
hold on
for i=1:10:T
  drawArrow([x(1,i) x(1,i)+arrow_mag*cos(x(4,i))], [x(2,i) x(2,i)+arrow_mag*sin(x(4,i))]);
end
plot(target(1), target(2), 'o', 'MarkerSize', marker_size);
text(target(1), target(2), 'TARGET')
plot(x(1, 1), x(1,2), 'x', 'MarkerSize', marker_size);
text(x(1, 1), x(1,2), 'START')
xlabel('x [m] ')
ylabel('y [m]')
grid on

figure()
plot3(x(1,:), x(2, :), x(3, :))
hold on
% for i=1:10:T
%   drawArrow3([x(1,i) x(1,i)+arrow_mag*cos(x(4,i))], [x(2,i) x(2,i)+arrow_mag*sin(x(4,i))], [x(3, i) x(3, i)]);
% end
plot3(target(1), target(2), target(3), 'o', 'MarkerSize', marker_size);
text(target(1), target(2), target(3), 'TARGET')
plot3(x(1, 1), x(1,2), x(1, 3), 'x', 'MarkerSize', marker_size);
text(x(1, 1), x(1,2), x(1, 3), 'START')
xlabel('x [m] ')
ylabel('y [m]')
zlabel('z [m]')
grid on

figure()
plot(t_vect, x(3, :))
xlabel('Time [s]')
ylabel('Vertical dynamic [m]')
grid on

figure()
plot(t_vect, x(1,:), 'r', 'LineWidth',line_width, 'DisplayName','x')
hold on
plot(t_vect, x_est(1,:), 'r--', 'LineWidth',line_width, 'DisplayName','x est')
plot(t_vect, x(2,:), 'g', 'LineWidth',line_width, 'DisplayName','y')
plot(t_vect, x_est(2,:), 'g--', 'LineWidth',line_width, 'DisplayName','y est')
plot(t_vect, x(3,:), 'k', 'LineWidth',line_width, 'DisplayName','z')
plot(t_vect, x_est(3,:), 'k--', 'LineWidth',line_width, 'DisplayName','z est')
plot(t_vect, x(4,:), 'b', 'LineWidth',line_width, 'DisplayName','$\theta$')
plot(t_vect, x_est(4,:), 'b--', 'LineWidth',line_width, 'DisplayName','$\theta$ est')
legend()

figure()
plot(t_vect, e_theta*180/pi, 'LineWidth',line_width)
xlabel('Time [s]')
ylabel('Angula error')