clc
close all
clear

% load the parameters
parameters_single_agent_M1;

%% Initialization
% State matrices 
A = eye(states_len);               % model-dependent matrix
B = eye(states_len, inputs_len)*dt; % model-dependent matrix
G = eye(states_len, states_len);   % disturbance matrix

% Input covariance matrix
Q = Q_scale*(rand(states_len, states_len) - Q_bias);
Q = Q*Q';

% Disturbances covariance matrix
L = L_scale*(rand(states_len, states_len) - L_bias);
L = L*L';
nu = zeros(states_len, 1);          % noise on the model


% Covariance matrix for the uncertainty
R_GPS = cell(1, n);
for i=1:n
  R_GPS = R_scale*(rand(measure_len, measure_len) - R_bias); % 
  R_GPS = R_GPS*R_GPS';
end

% States, inputs and state estimations
x = zeros(states_len, T);           % state
x(:, 1) = x0;                       % initialize the states
x_est = zeros(states_len, T);       % state estimation
x_est(:, 1) = x(:, 1);              % initialize state estimation
P_est = zeros(states_len);          % covaraince of estimation error
H_GPS = zeros(measure_len, states_len);
u = zeros(inputs_len, T);           % input
u_bar = zeros(inputs_len, T);           % input and its own noise

% LQR algorithm
K  = lqr(A, B, S, R, T, Sf);

for t=1:T-1
  
  nu(:) = mvnrnd(zeros(states_len, 1), L)';  % external disurbances

  % Optimal input
  u(:,t) = -K{i}*(x_est(:,t) - target);
  u_bar(:, t) = u(:, t) + mvnrnd(zeros(inputs_len, 1), Q)'; % input and its own noise 

  % Update the state
  x(:, t + 1) = A*x(:, t) + B*u(:, t) + G*nu;
  z_GPS = x(:, t + 1) + mvnrnd(zeros(inputs_len, 1), R_GPS)'; 
  [x_est, P_est] = kalman_filter(x_est, P_est, z_GPS, R_GPS, A, B, G, u_bar, Q, H_GPS, L, t, states_len);

  % Check if we have touch the ground
  if x(3, t) < 0
    x(:, t + 1) = 0;
    break
  end
end

%% Plots
drawArrow = @(x,y) quiver( x(1),y(1),x(2)-x(1),y(2)-y(1),0 );
drawArrow3 = @(x,y, z) quiver( x(1),y(1),x(2)-x(1),y(2)-y(1),z);
arrow_mag = 5;

figure(1); clf;
plot(t_vect, x(1,:), 'r', 'LineWidth',line_width, 'DisplayName','x')
hold on
plot(t_vect, x_est(1,:), 'r--', 'LineWidth',line_width, 'DisplayName','x est')
plot(t_vect, x(2,:), 'g', 'LineWidth',line_width, 'DisplayName','y')
plot(t_vect, x_est(2,:), 'g--', 'LineWidth',line_width, 'DisplayName','y est')
plot(t_vect, x(3,:), 'k', 'LineWidth',line_width, 'DisplayName','z')
plot(t_vect, x_est(3,:), 'k--', 'LineWidth',line_width, 'DisplayName','z est')
legend()
grid on

figure(2); clf;
plot(x(1,:), x(2, :))
hold on
plot(target(1), target(2), 'o', 'MarkerSize', marker_size);
text(target(1), target(2), 'TARGET')
plot(x(1, 1), x(1,2), 'x', 'MarkerSize', marker_size);
text(x(1, 1), x(1,2), 'START')
xlabel('x [m] ')
ylabel('y [m]')
grid on

figure(3); clf;
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

figure(4); clf;
plot(t_vect, x(3, :))
xlabel('Time [s]')
ylabel('Vertical dynamic [m]')
grid on


