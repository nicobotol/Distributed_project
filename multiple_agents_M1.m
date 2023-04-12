clc
close all
clear

% load the parameters
parameters

%% Initialization
% State matrices 
A = eye(states_len);               % model-dependent matrix
B = eye(states_len, inputs_len)*dt; % model-dependent matrix

%% Initialization
Qi = cell(1, n);    % Input covariance matrix
nu = cell(1, n);    % noise on the model
G = cell(1, n);     % disturbance matrix
R_GPS = cell(1, n); % Covariance amtrix for the uncertainty
for i=1:n
  Q{i} = Q_scale*(rand(states_len, states_len) - Q_bias);
  Q{i} = Q{i}*Q{i}';

  nu{i} = zeros(states_len, 1);    

  G{i} = eye(states_len, states_len);

  R_GPS{i} = R_GPS_scale*(rand(measure_len, measure_len) - R_GPS_bias);
  R_GPS{i} = R_GPS{i}*R_GPS{i}';

end

% States, inputs and state estimations
x = cell(1, n);                     % state
x_est = cell(1, n);                 % state estimation
P_est = cell(1, n);                 % covaraince of estimation error
H_GPS{i} = cell(1, n);              % model of the GPS
for i=1:n
  x{i} = zeros(states_len, T);
  x{i}(:, 1) = x0(i, :);            % initialize the states
  x_est{i} = zeros(states_len, T);
  x_est{i}(:, 1) = x{i}(:, 1);      % initialize state estimation
  P_est{i} = zeros(states_len);
  H_GPS{i} = zeros(measure_len, states_len);
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

K = cell(1, n);
u = cell(1, n);

for t=1:T-1
  for i=1:n
    nu{i}(:) = mvnrnd(zeros(states_len, 1), Q{i})';  % noise on the input and on the model
    % Optimal input
    K{i} = inv(R + B'*P{i}(:,:,t+1)*B)*B'*P{i}(:,:,t+1)*A; 
    u{i}(:,t) = -K{i}*(x{i}(:,t) - target);

    % Update the state
    x{i}(:, t+1) = A*x{i}(:, t) + B*u{i}(:, t) + nu{i};

    % Prediction
    x_est{i}(:, t+1) = A*x_est{i}(:, t) + B*u{i}(:, t);
    P_est{i} = A*P_est{i}*A' + G{i}*Q{i}*G{i};
  end

  % Update
  F = cell(n, 1);
  a = cell(n, 1); 
  for i =1:n
    F{i} = H_GPS{i}'*inv(R_GPS{i} + H_GPS{i}*P_est{i}*H_GPS{i}')*H_GPS{i};
    a{i} = H_GPS{i}'*inv(R_GPS{i} + H_GPS{i}*P_est{i}*H_GPS{i}')*zi;
  end
 %% ARRIVED HERE 

  % Measurement update using the GPS
  z_GPS = x(:, t + 1) + mvnrnd(zeros(inputs_len, 1), R_GPS)'; % measurement
  Innovation = z_GPS - x_est(:, t + 1);
  H_GPS = eye(states_len);  % linearized model of the GPSs
  % update the kalaman estimate
  S_Inno = H_GPS*P_est*H_GPS' + R_GPS;
  W = P_est*H_GPS'*inv(S_Inno); % kalman gain
  x_est(:, t + 1) = x_est(:, t + 1) + W*Innovation; % update stte estimate
  P_est = (eye(states_len) - W*H_GPS)*P_est; % update covariance matrix

  % Check if we have touch the ground
  if x(3, t) < 0
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


