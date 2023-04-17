clear all;
close all;
clc;

%% Initialization
set(0,'DefaultFigureWindowStyle','docked');
rng(3)
method = 'ddp';
addpath('functions/');

linewidth = 2;
marker_size = 10;

dt = 0.1; % [s]
sim_t = 100; % [s]
I = 1; % inertia
T = sim_t/dt; % number of iterations
t_vect = dt:dt:sim_t;
max_iter = 1000;
max_line_search = 10;
V_z = -1; % [m/s]

x = zeros(4,T); % chute position
u = zeros(4,T); % input matrix
u(3,:) = V_z;
initial = [30 30 70 pi];
x(:, 1) = initial; % initial state
x_est = zeros(4,T);
x_est(:,1) = x(:,1);
P_est = zeros(4,4);
target = [0,0,0,0]';
n = length(target);

% DDP init
x_bar = zeros(4,T);
u_bar = zeros(4,T);

% State covariance matrix
R_GPS = rand(4,4)-0.5;
R_GPS = R_GPS*R_GPS'; % bisogna cambiare l'incertezza di theta perche rad

% Input covariance matrix
Q = 0.1*(rand(4,4)-0.5);
Q = Q*Q';
nu = zeros(4, 1);

A = eye(4,4);
B = cell(1,T);
G = eye(4,4); % state 

% Cost matrices
S = eye(4);
R = eye(4,4);
Sf = eye(4);

if strcmp(method, 'lqr')
    %% Pseudo-LQR
    
    % Initialization
    P = cell(1,T);
    P{T} = Sf;
    
    % Kalman Filter
    for t=1:T-1
        nu(:) = mvnrnd([0;0;0;0], Q)';  % noise on the input and on the model 
        
        % LQR algorithm
        for i=T:-1:2
          B{i} = dt*[cos(x_est(4,i)), sin(x_est(4,i)), 0, 0;
            sin(x_est(4,i)), cos(x_est(4,i)), 0, 0;
             0, 0, 1, 0;
             0, 0, 0, 1];
          P{i-1} = S+A'*P{i}*A-A'*P{i}*B{i}*inv(R+B{i}'*P{i}*B{i})*B{i}'*P{i}*A;
        end
    
        % Optimal input
        K = inv(R+B{i}'*P{t+1}*B{i})*B{i}'*P{t+1}*A;
        u([1 2 4],t) = -K([1 2 4],[1 2 4])*(x([1 2 4],t)-target([1 2 4]));
    
        % State Update
        x(:, t+1) = A*x(:,t)+B{i}*u(:,t)+nu;
        
        % Predictions
        x_est(:, t+1) = A*x_est(:,t)+B{i}*u(:,t); % manca l'incertezza sul modello
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
           x(:,t:T) = zeros(4,T-t+1);
            break
        end
    end
elseif strcmp(method, 'ddp')

    %% DDP
    
    % Initialization
    x_bar = zeros(n, T);
    u_bar = zeros(n, T);
    d1 = 0;
    d2 = 0;
    mu = 1e-3;
    min_cost_impr = 1e-1;
    alpha_factor = 0.5;
    mu_factor = 10;
    mu_max = 10;
    min_alpha_to_increase_mu = 0.5;
    exp_impr_th = 1e-3;
    converged = false;
    K = cell(1,T);
    for i=1:T
        K{i} = zeros(4,4);
    end

    for j=1:max_iter
        % Dynamic Simulation
        [x_bar, u_bar] = sim_dyn(x_bar, u_bar, n, T, initial, K, V_z, dt);
        
        % Backward Pass
        [w_bar, K, Q_u, Q_uu] = backward_pass(x_bar,u_bar, mu, S, Sf, R, T, target, dt);

        % Forward Pass
        alpha = 1;
        line_search = false;
        l_f = (x_bar(:,end)-target)'*Sf*(x_bar(:,end)-target);
        cost = l_f;
        for p=1:T
            cost = cost + running_cost(p,x_bar,u_bar, S, R, target);
        end
        for s=1:T-1
            d1 = d1 + w_bar(:,s)'*Q_u{s};
            d2 = d2 + 0.5*w_bar(:,s)'*Q_uu{s}*w_bar(:,s);
        end

        for jj=1:max_line_search

            [x_new,u_new]  = sim_dyn(x_bar, u_bar+alpha*w_bar, n, T, initial, K, V_z, dt);
            new_cost = l_f;
            for p=1:T
                new_cost = new_cost + running_cost(p,x_new,u_new, S, R, target);
            end
            exp_impr = alpha*d1+0.5*alpha^2*d2;
            if exp_impr > 0
                exp_impr = -1;
            end
            rel_impr = (new_cost-cost)/exp_impr;
            
            if rel_impr > min_cost_impr
                fprintf("Cost improved from %.3f to %.3f. Exp. impr %.3f. Rel. impr. %.1f\n" , cost, new_cost, exp_impr, 1e2*rel_impr);
                line_search = true;
            end

            if line_search
                u_bar = u_bar + alpha*w_bar;
                cost = new_cost;
                break
            else 
                alpha = alpha_factor*alpha;
            end
        end

        if not(line_search)
            mu = mu*mu_factor;
            fprintf("No cost improvement, increasing mu to %f\n", mu);
            if mu>mu_max
                fprintf('Max regularization reached. Algorithm failed to converge\n');
                converged = true;
            end
            
        else
            fprintf("Line search succeded with alpha %f\n", alpha);
            if alpha > min_alpha_to_increase_mu
                mu = mu/mu_factor;
                fprintf("Decreasing mu to %f\n", mu);
            else
                mu  = mu*mu_factor;
                fprintf("Alpha is small: increasinf mu to %f\n", mu);
                if mu>mu_max
                    fprintf('Max regularization reached. Algorithm failed to converge\n');
                    converged = true;
                end
            end
        end
        
        if (abs(exp_impr) < exp_impr_th)
            fprintf("Algorothm converged. Expected improvment %f\n", exp_impr);
        end

        if converged
            break
        end

   
    end

    [x_bar, u_bar] = sim_dyn(x_bar, u_bar, n, T, initial, K, V_z, dt);
    
    % Kalman Filter
    for t=1:T-1
        nu(:) = mvnrnd([0;0;0;0], Q)';  % noise on the input and on the model 

        % State Update
        x(:, t+1) = x_bar(:, t+1)+nu;
        
        % Linearization
        A_t = eye(4);
        A_t(:,4) = [-u_bar(1,i)*sin(x_bar(4,i)*dt+u_bar(2,i)*cos(x_bar(4,i))*dt);
                u_bar(1,i)*cos(x_bar(4,i))*dt-u_bar(2,i)*sin(x_bar(4,i))*dt;
                0;
                1];
        
        % Predictions
        x_est(:, t+1) = x_bar(:, t+1);
        P_est = A_t*P_est*A_t' + G*Q*G';
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
            break
        end
    end

else
    fprintf("Wrong optimization method inserted!\n");
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
plot(t_vect, u(4,:), 'b', 'LineWidth', linewidth)
grid on
legend('V_{x}', 'V_{y}', 'V_{z}','\omega', 'Location','best')

if strcmp(method, 'ddp')
    figure(6)
    plot(t_vect, cost)
end






