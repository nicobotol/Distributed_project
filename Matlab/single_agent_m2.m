clear all;
close all;
clc;

%% Initialization
set(0,'DefaultFigureWindowStyle','docked');
rng(3)

linewidth = 2;
marker_size = 10;

dt = 0.01; % [s]
sim_t = 100; % [s]
I = 1; % inertia
T = sim_t/dt; % number of iterations
t_vect = dt:dt:sim_t;
V_z = 1; % [m/s]

x = ones(3,T); % chute position
u = zeros(3,T); % input matrix
u(3,:) = V_z;
x(:, 1) = [30 30 70]; % initial state
x_est = zeros(3,T);
x_est(:,1) = x(:,1);
P_est = zeros(3,3);
target = [0,0,0]';

% State covariance matrix
R_GPS = rand(3,3)-0.5;
R_GPS = R_GPS*R_GPS'; % bisogna cambiare l'incertezza di theta perche rad

% Input covariance matrix
Q = 0.1*(rand(3,3)-0.5);
Q = Q*Q';
nu = zeros(3, 1);

A = eye(3,3);
B = [dt 0 0;
     0 dt 0;
     0 0 -dt];
G = eye(3,3); % state 

%% LQR

S = [1 0 0;
      0 1 0;
      0 0 1];
R = 1.5*eye(3,3);
P = cell(1,T);
Sf = [1 0 0;
      0 1 0;
      0 0 1];
P{T} = Sf;

% LQR algorithm
for i=T:-1:2
    P{i-1} = S+A'*P{i}*A-A'*P{i}*B*inv(R+B'*P{i}*B)*B'*P{i}*A;
end

%% Kalman Filter

for t=1:T-1
    nu(:) = mvnrnd([0;0;0], Q)';  % noise on the input and on the model 

    % Optimal input
    K = inv(R+B'*P{t+1}*B)*B'*P{t+1}*A;
    u(1:2,t) = -K(1:2,1:2)*(x_est(1:2,t)-target(1:2));

    % State Update
    x(:, t+1) = A*x(:,t)+B*u(:,t)+nu;
    
    % Predictions
    x_est(:, t+1) = A*x_est(:,t)+B*u(:,t);
    P_est = A*P_est*A' + G*Q*G';
    % Measurements update
    z_GPS = x(:,t+1) + mvnrnd([0;0;0], R_GPS)';
    Innovation = z_GPS - x_est(:,t+1);
    H_GPS = eye(3,3);
    % Updated Kalman estimates
    S_Inno = H_GPS*P_est*H_GPS' + R_GPS; % covariance of inno
    W = P_est*H_GPS'*inv(S_Inno); % kalman filter gain
    x_est(:,t+1) = x_est(:,t+1) + W*Innovation; % updated state estimate
    P_est = (eye(3) - W*H_GPS)*P_est; % updated cov matrix
    
    if x(3,t) < 0
       x(:,t:T) = zeros(3,T-t+1);
        break
    end
end

%% Plot

figure(1)
plot(t_vect, x(1,:), 'r', 'LineWidth', linewidth)
hold on
plot(t_vect, x_est(1,:), 'r--', 'LineWidth', linewidth)
plot(t_vect, x(2,:), 'g', 'LineWidth', linewidth)
plot(t_vect, x_est(2,:), 'g--', 'LineWidth', linewidth)
plot(t_vect, x(3,:), 'k', 'LineWidth', linewidth)
plot(t_vect, x_est(3,:), 'k--', 'LineWidth', linewidth)
grid on
legend('x', 'x_{est}', 'y', 'y_{est}', 'z', 'z_{est}', 'Location','best')

figure(2) 
plot(x(1,:), x(2, :)) 
hold on 
plot(target(1), target(2), 'o', 'MarkerSize', marker_size); 
text(target(1), target(2), 'TARGET') 
plot(x(1, 1), x(1,2), 'x', 'MarkerSize', marker_size); 
text(x(1, 1), x(1,2), 'START') 
xlabel('x [m] ') 
ylabel('y [m]') 
grid on 
 
figure(3) 
plot3(x(1,:), x(2, :), x(3, :)) 
hold on 
plot3(target(1), target(2), target(3), 'o', 'MarkerSize', marker_size); 
text(target(1), target(2), target(3), 'TARGET') 
plot3(x(1, 1), x(2,1), x(3, 1), 'x', 'MarkerSize', marker_size); 
text(x(1, 1), x(2,1), x(3, 1), 'START') 
xlabel('x [m] ') 
ylabel('y [m]') 
zlabel('z [m]') 
grid on 
 
figure(4) 
plot(t_vect, x(3, :)) 
xlabel('Time [s]') 
ylabel('Vertical dynamic [m]') 
grid on

figure(5)
plot(t_vect, u(1,:), 'r', 'LineWidth', linewidth)
hold on
plot(t_vect, u(2,:), 'g', 'LineWidth', linewidth)
plot(t_vect, u(3,:), 'k', 'LineWidth', linewidth)
grid on
legend('V_{x}', 'V_{y}', 'V_{z}', 'Location','best')