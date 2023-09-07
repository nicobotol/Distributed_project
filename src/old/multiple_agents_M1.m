clc
close all
clear

% load the parameters
parameters_multiple_agents_M1

%% Initialization
% State matrices 
A = eye(states_len);               % model-dependent matrix
B = eye(states_len, inputs_len)*dt; % model-dependent matrix
G = eye(states_len, states_len);

%% Initialization
Qi = cell(1, n);    % Input covariance matrix
nu = cell(1, n);    % noise on the model
z_GPS = zeros(states_len, n); % GPS measurements
R_GPS = cell(1, n); % Covariance amtrix for the uncertainty
L_val = L_scale*(rand(states_len, states_len) - L_bias);
L_val = L_val*L_val';
for i=1:n
  % Input covariance matrix
  Q{i} = Q_scale*(rand(states_len, states_len) - Q_bias);
  Q{i} = Q{i}*Q{i}';

  % noise on the model
  nu{i} = zeros(states_len, 1);    

  % Disturbances covariance matrix
  L{i} = L_val;

  R_GPS{i} = R_GPS_scale*(rand(states_len, states_len) - R_GPS_bias);
  R_GPS{i} = R_GPS{i}*R_GPS{i}';

end

% States, inputs and state estimations
x = cell(1, n);                     % state
x_est = cell(1, n);                 % state estimation
P_est = cell(1, n);                 % covaraince of estimation error
H_GPS{i} = cell(1, n);              % model of the GPS
K = cell(1, n); % each element of the cell is a matrix
u = cell(1, n); % LQR input; each cell element is a matrix collecting input's hystory
u_bar = cell(1, n);

for i=1:n
  x{i} = zeros(states_len, T);
  x{i}(:, 1) = x0(:, i);            % initialize the states
  x_est{i} = zeros(states_len, T);
  x_est{i}(:, 1) = x{i}(:, 1);      % initialize state estimation
  P_est{i} = zeros(states_len);
  H_GPS{i} = eye(states_len, states_len);
  x_centroid = zeros(states_len, T);% position of the centroid 
end

% Matrices used in the KF
S_Inno = cell(1, n);
Innovation = cell(1, n);
W = cell(1, n);
for i = 1:n
  S_Inno{i} = zeros(states_len);
  Innovation{i} = zeros(states_len, 1);
  W{i} = zeros(states_len);
end

%% LQR CONTROL
K = lqr(A, B, S, R, T, Sf, n, states_len);

for t=1:T-1
  if x{i}(3, t) > target(3) % Check if we have touch the ground
    
    for i=1:n
      % LQR input
      u{i}(:, t) = -K{i}(:,:,t)*(x_est{i}(:, t) - target);
      u_bar{i}(:, t) = u{i}(:, t) + mvnrnd(zeros(inputs_len, 1), Q{i})';
      % Update the state
      nu{i} = mvnrnd(zeros(states_len, 1), L{i})';  % noise on the input and on the model
      x{i}(:, t + 1) = A*x{i}(:, t) + B*u{i}(:, t) + G*nu{i};
      % Measurementusing the GPS
      z_GPS(:, i) = x{i}(:, t + 1) + mvnrnd(zeros(inputs_len, 1), R_GPS{i})';
    end

    % Kalman filter
    [x_est, P_est] = kalman_filter(x_est, P_est, z_GPS, R_GPS, A, B, G, u_bar, Q, H_GPS, L, t, states_len);
  end
end

%   for i=1:n
%     if x{i}(3, t) > target(3) % Check if we have touch the ground

%       nu{i}(:) = mvnrnd(zeros(states_len, 1), Q{i})';  % noise on the input and on the model
%       % Optimal input
%       K{i} = inv(R + B'*P{i}(:, :, t + 1)*B)*B'*P{i}(:, :, t + 1)*A; 
%       u{i}(:, t) = -K{i}*(x_est{i}(:, t) - target);
  
%       % Update the state
%       x{i}(:, t + 1) = A*x{i}(:, t) + B*u{i}(:, t) + nu{i};
  
%       % Prediction
%       x_est{i}(:, t+1) = A*x_est{i}(:, t) + B*u{i}(:, t);
%       P_est{i} = A*P_est{i}*A' + G{i}*Q{i}*G{i};
  
%       % Measurement update using the GPS
%       z_GPS(:, i) = x{i}(:, t + 1) + mvnrnd(zeros(inputs_len, 1), R_GPS{i})'; % measurement
%       Innovation{i} = z_GPS(:, i) - x_est{i}(:, t + 1);
    
%       % update the kalaman estimate
%       S_Inno{i} = H_GPS{i}*P_est{i}*H_GPS{i}' + R_GPS{i};
%       W{i} = P_est{i}*H_GPS{i}'*inv(S_Inno{i}); % kalman gain
%       x_est{i}(:, t + 1) = x_est{i}(:, t + 1) + W{i}*Innovation{i}; % update stte estimate
%       P_est{i} = (eye(states_len) - W{i}*H_GPS{i})*P_est{i}; % update covariance matrix
%     end
%   end

% end

%% Plots
drawArrow = @(x,y) quiver( x(1),y(1),x(2)-x(1),y(2)-y(1),0 );
drawArrow3 = @(x,y, z) quiver( x(1),y(1),x(2)-x(1),y(2)-y(1),z);
arrow_mag = 5;

figure(2); clf;
hold all
for i=1:n
  plot(x{i}(1,:), x{i}(2, :), 'DisplayName', ['Agent ', num2str(i)])
  plot(x{i}(1, 1), x{i}(1,2), 'x', 'MarkerSize', marker_size, ...
    'DisplayName', ['START', num2str(i)]);
end
plot(target(1), target(2), 'o', 'MarkerSize', marker_size, ...
  'DisplayName', 'TARGET');
xlabel('x [m] ')
ylabel('y [m]')
legend('Location', 'best')
grid on

figure(3); clf;
hold all
for i=1:n
  plot3(x{i}(1,:), x{i}(2, :), x{i}(3, :), 'DisplayName', ['Agent ', num2str(i)])
  plot3(x{i}(1, 1), x{i}(1,2), x{i}(3, 1), 'x', 'MarkerSize', ...
    marker_size, 'DisplayName', ['START', num2str(i)]);
end
plot3(target(1), target(2), target(3), 'o', 'MarkerSize', marker_size, ...
  'DisplayName', 'TARGET');
plot3(x_centroid(1, :), x_centroid(2,:), x_centroid(3, :), 'r--', 'DisplayName', 'Centroid')
xlabel('x [m] ')
ylabel('y [m]')
zlabel('z [m]')
legend('Location', 'best')
grid on

figure(4); clf;
hold all
for i=1:n
  plot(t_vect, x{i}(3, :), 'DisplayName',strcat('Agent ', num2str(i)))
end
xlabel('Time [s]')
ylabel('Vertical dynamic [m]')
grid on


