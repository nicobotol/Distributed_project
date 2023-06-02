clear all;
close all;
clc;

%% Initialization
set(0,'DefaultFigureWindowStyle','docked');
rng(3)

% this model is a PID low level controller to move the drone to the new local centroid without overshooting it (the lqr doesn't guarantee that)
linewidth = 2;
marker_size = 10;

dt = 0.1; % [s]
sim_t = 200; % [s]
T = sim_t/dt; % number of iterations
t_vect = dt:dt:sim_t;
V_z = -25; % [m/s]
V = 10; % [m/s] wind speed

x = zeros(4,T); % chute position
u = zeros(2,T); % input
nu = zeros(6,T); % non controllable input
initial_2D = [500 500 0];
initial_3D = [500 500 3000 0];
x(:, 1) = initial_3D; % initial state
x_est = zeros(4,T);
x_est(:,1) = x(:,1);
P_est = zeros(4,4);
target = [0,0]';
[n,~] = size(x);

% State covariance matrix
R_GPS = 0.1*(rand(4,4)-0.5);
R_GPS = R_GPS*R_GPS'; % bisogna cambiare l'incertezza di theta perche rad

% Input covariance matrix
Q = 0.1*(rand(2,2)-0.5);
Q = Q*Q';

% Distrubances covariance matrix
L = 1*(rand(6,6)-0.5);
L = L*L';

A = eye(4,4);
B =[0;0;0;dt];
G = cell(1, T);
G_est = cell(1, T);
    
% Kalman Filter
for t=1:T-1
    nu(:,t) = 0.1*randn(6,1);
    nu(1,:) = V;
    nu(5,:) = V_z;
    nu_unc(:,t) = nu(:,t) + mvnrnd([0;0;0;0;0;0], L)';  % noise on the non controllable inputs
    
    % % Angle of the chute wrt the x axis
    % if x_est(1,t) - target_2D(1) > 0 && x_est(2,t) - target_2D(2) > 0
    %     alpha(t) = atan2(x_est(2,t) - target_2D(2), x_est(1,t) - target_2D(1));          
    % elseif x_est(1,t) - target_2D(1) > 0 && x_est(2,t) - target_2D(2) < 0
    %     alpha(t) = atan2(x_est(2,t) - target_2D(2), x_est(1,t) - target_2D(1)) + 2*pi;  
    % elseif x_est(1,t) - target_2D(1) < 0 && x_est(2,t) - target_2D(2) > 0
    %     alpha(t) = atan2(x_est(2,t) - target_2D(2), x_est(1,t) - target_2D(1));      
    % elseif x_est(1,t) - target_2D(1) < 0 && x_est(2,t) - target_2D(2) < 0
    %     alpha(t) = atan2(x_est(2,t) - target_2D(2), x_est(1,t) - target_2D(1)) + 2*pi;
    % end

    % % Desired angle of the chute wrt the x axis
    % theta_des = alpha(t) + pi/2;
    % x_est(4,t) = wrapTo2Pi(x_est(4,t));
    % x(4,t) = wrapTo2Pi(x(4,t));

    % PID controller
    K_v = 0.1;
    K_omega = 0.1;

    u(1,t) = K_v*max(0,min(V, [cos(x_est(4,t)) sin(x_est(4,t))]*(target - x_est(1:2,t))));
    u(2,t) = K_omega*atan2([-sin(x_est(4,t)) cos(x_est(4,t))]*(target - x_est(1:2,t)),[cos(x_est(4,t)) sin(x_est(4,t))]*(target - x_est(1:2,t)));

    u_unc(:,t) = u(:,t) +  mvnrnd(0, Q)'; % noise on the inputs
    
    % State Update
    G{t} = [-sin(x(4,t))*dt 1 0 0 0 0;
            cos(x(4,t))*dt 0 1 0 0 0;
            0 0 0 1 dt 0;
            0 0 0 0 0 1];
    x(:, t+1) = A*x(:,t)+B*u_unc(:,t)+G{t}*nu_unc(:,t);
    
    % Predictions
    G_est{t} = [-sin(x_est(4,t))*dt 0 0 0 0 0;
            cos(x_est(4,t))*dt 0 0 0 0 0;
            0 0 0 0 dt 0;
            0 0 0 0 0 0];
    x_est(:, t+1) = A*x_est(:,t)+B*u(:,t)+G_est{t}*nu(:,t); % non conosciamo nu se non lo misuriamo 
    P_est = A*P_est*A' + B*Q*B' + G_est{t}*L*G_est{t}';
    % Measurements update
    z_GPS = x(:,t+1) + mvnrnd([0;0;0;0], R_GPS)';
    Innovation = z_GPS - x_est(:,t+1);
    H_GPS = eye(4,4);
    % Updated Kalman estimates
    S_Inno = H_GPS*P_est*H_GPS' + R_GPS; % covariance of inno
    W = P_est*H_GPS'*inv(S_Inno); % kalman filter gain
    x_est(:,t+1) = x_est(:,t+1) + W*Innovation; % updated state estimate
    P_est = (eye(4) - W*H_GPS)*P_est; % updated cov matrix
    
    if x(3,t) < 0
        x(:,t:T) = zeros(4,T-t+1);
        x_est(:,t:T) = zeros(4,T-t+1);
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
plot(t_vect, x(4,:), 'b', 'LineWidth', linewidth)
plot(t_vect, x_est(4,:), 'b--', 'LineWidth', linewidth)
grid on
legend('x', 'x_{est}', 'y', 'y_{est}', 'z', 'z_{est}',  '\theta', '\theta_{est}','Location','best')

figure(2) 
plot(x_est(1,:), x_est(2, :)) 
hold on
% for i=1:10:T
%     anArrow_x = annotation('arrow');
%     anArrow_x.Parent = gca;
%     anArrow_x.X = [x(1,i),x(1,i)+5*cos(x(4,i))]; % set the x-property
%     anArrow_x.Y = [x(2,i) ,x(2,i)+5*sin(x(4,i))];
%     anArrow_x.Color = 'black';     

%     anArrow_y = annotation('arrow');
%     anArrow_y.Parent = gca;
%     anArrow_y.X = [x(1,i),x(1,i)+5*cos(x(4,i)+pi/2)]; % set the x-property
%     anArrow_y.Y = [x(2,i) ,x(2,i)+5*sin(x(4,i)+pi/2)];
%     anArrow_y.Color = 'black';    
% end 
plot(target(1), target(2), 'o', 'MarkerSize', marker_size); 
text(target(1), target(2), 'TARGET') 
plot(x(1, 1), x(1,2), 'x', 'MarkerSize', marker_size); 
text(x(1, 1), x(1,2), 'START') 
xlabel('x [m] ') 
ylabel('y [m]') 
grid on 
 
% calculate the z trajectory with constant velocity V_z
z = zeros(1,T+1);
z(1)= initial_3D(1,3);
for i=1:T
    z(i+1) = z(i) + V_z * dt * i;
    if z(i+1) < 0
        z(i+1:T+1) = zeros(1,T-i+1);
        break
    end
end

for t=1:T
    if z(t) == 0
        x_bar(:,t) = zeros(3,1);
    end
end

figure(3)
plot3(x(1,:), x(2, :), x(3,:))
hold on 
% plot3(target(1), target(2), target(3), 'o', 'MarkerSize', marker_size); 
% text(target(1), target(2), target(3), 'TARGET') 
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
plot(t_vect, V_z*ones(1,length(t_vect)), 'k--', 'LineWidth', linewidth)
plot(t_vect, V*ones(1,length(t_vect)), 'b--', 'LineWidth', linewidth)
grid on
legend('\omega','V_z','V', 'Location','best')

if strcmp(method, 'ddp')
    figure(6)
    plot(1:max_iter, cost(1,:))
end