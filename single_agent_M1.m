clc
close all
clear

% load the parameters
parameters

%% Initialization
% State matrices 
A = eye(states_len);               % model-dependent matrix
B = eye(states_len, inputs_len)*dt; % model-dependent matrix

% Input covariance matrix
Qi = Qi_scale*(rand(states_len, states_len) - Qi_bias);
Qi = Qi*Qi';
nu = zeros(states_len, 1);          % noise on the model

G = eye(states_len, states_len);   % disturbance matrix

% Covariance amtrix for the uncertainty
R_GPS = Ri_scale*(rand(measure_len, measure_len) - Ri_bias); % 
R_GPS = R_GPS*R_GPS';

% States, inputs and state estimations
x = zeros(states_len, T);           % state
u = zeros(inputs_len, T);           % input
up = zeros(inputs_len, T);          % proportional input
ui = zeros(inputs_len, T);          % integral input
ud = zeros(inputs_len, T);          % derivative input
x(:, 1) = x0;                       % initialize the states
x_est = zeros(states_len, T);       % state estimation
x_est(:, 1) = x(:, 1);              % initialize state estimation
P_est = zeros(states_len);          % covaraince of estimation error
H_GPS = zeros(measure_len, states_len);

P = cell(1, T);
P{T} = Sf;

% LQR algorithm
for i=T:-1:2
    P{i-1} = S+A'*P{i}*A-A'*P{i}*B*inv(R+B'*P{i}*B)*B'*P{i}*A;
end

for t=1:T-1
  
  nu(:) = mvnrnd(zeros(states_len, 1), Qi)';  % noise on the input and on the model

  % Optimal input
  K = inv(R+B'*P{t+1}*B)*B'*P{t+1}*A; 
  u(:,t) = -K*(x(:,t)-target);

  % Update the state
  x(:, t+1) = A*x(:, t) + B*u(:, t) + nu;
  
  % Prediction
  x_est(:, t+1) = A*x_est(:, t) + B*u(:, t);

  P_est = A*P_est*A' + G*Qi*G'; 

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


