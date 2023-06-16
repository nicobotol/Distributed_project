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
V_z = 10; % [m/s]

x = ones(3,T); % chute position
u = zeros(2,T); % input matrix
x(:, 1) = [30 30 70]; % initial state
x_est = zeros(3,T);
x_est(:,1) = x(:,1);
P_est = zeros(3,3);
target = [0,0,0]';

% State covariance matrix
R_GPS = rand(3,3)-0.5;
R_GPS = R_GPS*R_GPS'; % bisogna cambiare l'incertezza di theta perche rad

% Input covariance matrix
Q = (rand(2,2)-0.5);
Q = Q*Q';

% Distrubances covariance matrix
L = 5*(rand(4,4)-0.5);
L = L*L';

A = eye(3,3);
B = [dt 0;
     0 dt;
     0 0 ];
G = eye(3,3); % disturbances matrix 
G(:,4) = [0; 0 ;dt]; % add the input to the disturbances

%% LQR

S = 5*[1 0 0;
      0 1 0;
      0 0 1];
R = eye(2,2);
P = cell(1,T);
Sf = 10*[1 0 0;
      0 1 0;
      0 0 1];
P{T} = Sf;

% LQR algorithm
for i=T:-1:2
    P{i-1} = S+A'*P{i}*A-A'*P{i}*B*inv(R+B'*P{i}*B)*B'*P{i}*A;
end

%% Kalman Filter

for t=1:T-1
    nu(:,t) = 0.1*randn(4,1);
    nu(4,:) = -V_z;
    nu_unc(:,t) = nu(:,t) + mvnrnd([0;0;0;0], L)';  % noise on the non controllable inputs 

    % Optimal input
    K = inv(R+B'*P{t+1}*B)*B'*P{t+1}*A;
    u(:,t) = -K*(x_est(:,t)-target);

    % State Update
    x(:, t+1) = A*x(:,t)+B*u(:,t)+G*nu(:,t);
    
    % Predictions
    x_est(:, t+1) = A*x_est(:,t)+B*u(:,t)+G*nu_unc(:,t);
    P_est = A*P_est*A' + B*Q*B' + G*L*G';
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
grid on
legend('V_{x}', 'V_{y}', 'Location','best')