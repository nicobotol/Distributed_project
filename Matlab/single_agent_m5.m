clear all;
close all;
clc;

%% Initialization
set(0,'DefaultFigureWindowStyle','docked');
rng(3)

% Add functions folder to path
addpath('functions')

% This model is a PID low level controller to move the drone to the new local centroid without overshooting it (the lqr doesn't guarantee that)
linewidth = 2;
marker_size = 10;

dt = 0.1;           % [s] why changing dt changes the results?
sim_t = 100;        % [s]
T = sim_t/dt;       % number of iterations
t_vect = dt:dt:sim_t;

beta = 0.01;        % [1/s] coefficient for the terminal velocity
v_lim = 1;          % [m/s] terminal velocity
V = 20;             % [m/s] wind speed
min_vel = 0.5;      % [m/s] minimum velocity

%% PID controller
K_v = 1;            % [1/s] velocity gain
K_omega = 2;        % [1/s] angular velocity gain

x = zeros(4,T);     % chute position
u = zeros(2,T);     % input
nu = zeros(5,T);    % non controllable input
initial_2D = [50 50 0];
initial_3D = [50 50 60 0];
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
L = 0.1*(rand(5,5)-0.5);
L = L*L';

A = eye(4,4);
G = eye(4,5);
G(:,4) = [0;0;dt;0];
    
for t=1:T-1

    V_z(t) = -v_lim*(1-exp(-beta*t)); % [m/s] vertical velocity

    % Control matrix
    B =dt*[cos(x(4,t)) 0;
    sin(x(4,t)) 0;
    0 0;
    0 1];
    
    % Estimated control matrix
    B_est =dt*[cos(x_est(4,t)) 0;
    sin(x_est(4,t)) 0;
    0 0;
    0 1];

    % Non controllable inputs
    nu(:,t) = 0.1*randn(5,1); 
    nu(3,t) = V_z(t);
    nu_unc(:,t) = nu(:,t) + mvnrnd([0;0;0;0;0], L)';  

    % Angle in the interval [0, 2*pi]
    x_est(4,t) = wrapTo2Pi(x_est(4,t));
    x(4,t) = wrapTo2Pi(x(4,t));

    % Optimal control
    % The control is bounded by the minimum velocity and the terminal velocity
    if [cos(x_est(4,t)) sin(x_est(4,t))]*(target - x_est(1:2,t)) > min_vel
        u(1,t) = K_v*min(V, [cos(x_est(4,t)) sin(x_est(4,t))]*(target - x_est(1:2,t)));
    else 
        u(1,t) = min_vel;
    end
    u(2,t) = K_omega*atan2([-sin(x_est(4,t)) cos(x_est(4,t))]*(target - x_est(1:2,t)),[cos(x_est(4,t)) sin(x_est(4,t))]*(target - x_est(1:2,t)));

    u_unc(:,t) = u(:,t) +  mvnrnd([0;0], Q)'; % noise on the inputs
    
    %% Extended Kalman Filter

    % Linearized model
    A_nl = eye(4);
    A_nl(:,4) = [-V*sin(x_est(4,t))*dt; V*cos(x_est(4,t))*dt; 0; 1]; 

    % State update
    x(:, t+1) = A*x(:,t)+B*u_unc(:,t)+G*nu_unc(:,t);
    
    % Predictions
    x_est(:, t+1) = A*x_est(:,t)+B_est*u(:,t)+G*nu(:,t); % non conosciamo nu se non lo misuriamo 
    P_est = A_nl*P_est*A_nl' + G*L*G';

    % Check whether the arriving point is inside the voronoi cell, if not move it to the closest point on the voronoi cell
    voronoicell = circle(x_est(1,t),x_est(2,t),1);
    true_voronoicell = circle(x(1,t),x(2,t),1);
    [in, on] = inpolygon(x_est(1,t+1), x_est(2,t+1), voronoicell(:,1), voronoicell(:,2));
    if ~in && ~on
        k = dsearchn(voronoicell, [x_est(1,t+1), x_est(2,t+1)]);
        x_est(1,t+1) = voronoicell(k,1);
        x_est(2,t+1) = voronoicell(k,2);
    end

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
        V_z(t:T) = zeros(1,T-t+1);
        break
    end

    % Animated plot
    figure(1);clf;
    hold all
    plot(x(1,t), x(2,t), 'o', 'MarkerSize', marker_size);
    plot(voronoicell(:,1), voronoicell(:,2), 'k');
    plot(x(1,t+1), x(2,t+1), 'x', 'MarkerSize', marker_size);
    xlabel('x [m] ')
    ylabel('y [m]')
    xlim([-50 50])
    ylim([-50 50])
    legend('actual position', 'voronoi cell', 'estimated position', 'Location','best')
    grid on
    axis equal
    drawnow
end

%% Plot

figure(2)
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

figure(3) 
plot(x_est(1,:), x_est(2, :)) 
hold on
plot(target(1), target(2), 'o', 'MarkerSize', marker_size); 
text(target(1), target(2), 'TARGET') 
plot(x(1, 1), x(1,2), 'x', 'MarkerSize', marker_size); 
text(x(1, 1), x(1,2), 'START') 
xlabel('x [m] ') 
ylabel('y [m]') 
grid on 

figure(4)
plot3(x(1,:), x(2, :), x(3,:))
hold on 
plot3(target(1), target(2), 0, 'o', 'MarkerSize', marker_size); 
text(target(1), target(2), 0, 'TARGET') 
plot3(x(1, 1), x(2,1), x(3, 1), 'x', 'MarkerSize', marker_size); 
text(x(1, 1), x(2,1), x(3, 1), 'START') 
xlabel('x [m] ') 
ylabel('y [m]') 
zlabel('z [m]') 
grid on 
 
figure(5) 
plot(t_vect, x(3, :)) 
xlabel('Time [s]') 
ylabel('Vertical dynamic [m]') 
grid on

figure(6)
plot(t_vect, u(1,:), 'r', 'LineWidth', linewidth)
hold on
plot(t_vect, u(2,:), 'b', 'LineWidth', linewidth)
plot(t_vect, V_z, 'k--', 'LineWidth', linewidth)
grid on
legend('v','\omega','V_z', 'Location','best')