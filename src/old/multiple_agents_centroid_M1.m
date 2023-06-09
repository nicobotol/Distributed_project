clc
close all
clear

% load the parameters
parameters_multiple_agents_M1

%% Initialization
% State matrices 
A = eye(states_len);               % model-dependent matrix
B = eye(states_len, inputs_len)*dt; % model-dependent matrix

%% Initialization
Qi = cell(1, n);    % Input covariance matrix
nu = cell(1, n);    % noise on the model
G = cell(1, n);     % disturbance matrix
z_GPS = zeros(states_len, n); % GPS measurements
R_GPS = cell(1, n); % Covariance amtrix for the uncertainty
for i=1:n
  Q{i} = Q_scale*(rand(states_len, states_len) - Q_bias);
  Q{i} = Q{i}*Q{i}';

  nu{i} = zeros(states_len, 1);    

  G{i} = eye(states_len, states_len);

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
x_centroid = zeros(states_len, T);
% x_centroid = cell(1, n);
for i=1:n
  x{i} = zeros(states_len, T);
  x{i}(:, 1) = x0(:, i);            % initialize the states
  x_est{i} = zeros(states_len, T);
  x_est{i}(:, 1) = x{i}(:, 1);      % initialize state estimation
  P_est{i} = zeros(states_len);
  H_GPS{i} = eye(states_len, states_len);
  % x_centroid{i} = zeros(states_len, T);% position of the centroid 
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
P = cell(1, n);
for i=1:n
  P{i} = zeros(states_len, states_len, T);
  P{i}(:, :, T) = Sf;
  % LQR algorithm
  for j=T:-1:2
    P{i}(:,:,j-1) = S + A'*P{i}(:,:,j)*A - A'*P{i}(:,:,j)*B*inv(R + ...
      B'*P{i}(:,:,j)*B)*B'*P{i}(:,:,j)*A;
  end
end

for t=1:T-1
  for i=1:n
    if x{i}(3, t) > target(3) % Check if we have touch the ground

      nu{i}(:) = mvnrnd(zeros(states_len, 1), Q{i})';  % noise on the input and on the model
      % Optimal input
      K{i} = inv(R + B'*P{i}(:, :, t + 1)*B)*B'*P{i}(:, :, t + 1)*A; 
      u{i}(:, t) = -K{i}*(x_est{i}(:, t) - target);
  
      % Update the state
      x{i}(:, t + 1) = A*x{i}(:, t) + B*u{i}(:, t) + nu{i};
  
      % Prediction
      x_est{i}(:, t+1) = A*x_est{i}(:, t) + B*u{i}(:, t);
      P_est{i} = A*P_est{i}*A' + G{i}*Q{i}*G{i};
  
      % Measurement update using the GPS
      z_GPS(:, i) = x{i}(:, t + 1) + mvnrnd(zeros(inputs_len, 1), R_GPS{i})'; % measurement
      Innovation{i} = z_GPS(:, i) - x_est{i}(:, t + 1);
    
      % update the kalaman estimate
      S_Inno{i} = H_GPS{i}*P_est{i}*H_GPS{i}' + R_GPS{i};
      W{i} = P_est{i}*H_GPS{i}'*inv(S_Inno{i}); % kalman gain
      x_est{i}(:, t + 1) = x_est{i}(:, t + 1) + W{i}*Innovation{i}; % update stte estimate
      P_est{i} = (eye(states_len) - W{i}*H_GPS{i})*P_est{i}; % update covariance matrix
    end
  end

              %   %% Linear consensus on the centroid position
              %   A = ones(n);     % TOPOLOGY MATRIX
              %   D = A*ones(n, 1); % vector of node degrees 
              %   Q = eye(n);
              %   for i=1:n
              %     for j=i+1:n
              %       Q(i, j) = 
              %     end
              %   end
  x_temp = zeros(states_len, n);
  for i=1:n
    x_temp(i, :) = x_est{i}(:, t + 1)'; 
  end
  A_cons = 1/n*ones(1, n);
  consensus = A_cons*x_temp;
  x_centroid(:, t +1 ) = consensus';
  
  % A_cons = 1/n*ones(n);
  % consensus = A_cons^m*x_temp;
  % for i=1:n
  %   x_centroid{i}(:, t + 1) = consensus(i, :)';
  % end
  
end
x_centroid(:, 1) = A_cons*x0'; % intiialize the first value of the centroid

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
    % plot3(x_centroid{i}(1, :), x_centroid{i}(2,:), x_centroid{i}(3, :), '--', ...
    %   'DisplayName', 'Centroid')
end
plot3(x_centroid(1, :), x_centroid(2,:), x_centroid(3, :), '--', ...
  'DisplayName', 'Centroid');
plot3(target(1), target(2), target(3), 'o', 'MarkerSize', marker_size, ...
  'DisplayName', 'TARGET');
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


