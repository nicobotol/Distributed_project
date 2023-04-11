clear all;
close all;
clc;

%% Initialization
set(0,'DefaultFigureWindowStyle','docked');
rng(3)

linewidth = 2;
marker_size = 10;

dt = 0.01; % [s]
sim_t = 10; % [s]
I = 1; % inertia
T = sim_t/dt; % number of iterations
t_vect = dt:dt:sim_t;
k_p = 10; % proportional control
V_x = 10;
V_z = 5;
e_theta = zeros(length(t_vect));

x = zeros(4,T); % chute position
u = zeros(3,T); % input matrix
x(:, 1) = [30 30 50 pi]; % initial state
x_est = zeros(4,T);
x_est(:,1) = x(:,1);
P_est = zeros(4,4);
target = [0,0,0]';

% State covariance matrix
R_GPS = rand(4,4)-0.5;
R_GPS = R_GPS*R_GPS'; % bisogna cambiare l'incertezza di theta perche rad

% Input covariance matrix
Q = 0.1*(rand(4,4)-0.5);
Q = Q*Q';
nu = zeros(4, 1);

A = eye(4,4);
G = eye(4,4); % state 


%% Kalman Filter

for t=1:T-1
    alpha = atan2(x_est(2,t)- target(2), x_est(1,t)- target(1));
    
    e_theta(t) = pi+alpha - x_est(4,t);
    m(t+1) = k_p*(e_theta(t)); % torque proportional to angular error
    u(:, t) = [V_x V_z m]';
    nu(:) = mvnrnd([0; 0; 0; 0], Q)';  % noise on the input and on the model 

    % State Update
    x(1, t+1) = x(1,t) + u(1,t)*cos(x(4,t))*dt + nu(1);
    x(2, t+1) = x(2,t) + u(1,t)*sin(x(4,t))*dt + nu(2);
    x(3, t+1) = x(3,t) - u(2,t)*dt + nu(3);
    x(4, t+1) = x(4,t) + u(3,t)*dt^2/(2*I) + nu(4);
    
    % Preditions
    % - Compute the preditions
    x_est(1, t+1) = x_est(1,t) + u(1,t)*cos(x(4,t))*dt;
    x_est(2, t+1) = x_est(2,t) + u(1,t)*sin(x(4,t))*dt;
    x_est(3, t+1) = x_est(3,t) - u(2,t)*dt;
    x_est(4, t+1) = x_est(4,t) + u(3,t)*dt^2/(2*I);
    % Linearized Ah(k) 
    Ah = eye(4); 
    Ah(1, 4) = -u(1, t)*sin(x_est(4, t))*dt; 
    Ah(2, 4) = u(1, t)*cos(x_est(4, t))*dt; 
    
    P_est = A*P_est*A' + G*Q*G';
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
        break
    end
end

%% Plot

figure()
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
legend('x', 'x_est', 'y', 'y_est', 'z', 'z_est', '$\theta$', '$\theta_est$', 'Location','best')

figure()
plot(t_vect, e_theta, 'LineWidth', linewidth);

drawArrow = @(x,y) quiver( x(1),y(1),x(2)-x(1),y(2)-y(1),0); 
figure() 
plot(x(1,:), x(2, :)) 
hold on 
for i=1:10:T 
  drawArrow([x(1,i) x(1,i)+0.1*cos(x(4,i))], [x(2,i) x(2,i)+0.1*sin(x(4,i))]); 
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