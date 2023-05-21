clear all;
close all;
clc;

%% Initialization
set(0,'DefaultFigureWindowStyle','docked');
rng(3)
% ask the user to choose the method
method = input('Choose the method: lqr or ddp: ', 's');
if ~(strcmp(method, 'lqr') || strcmp(method, 'ddp'))
    error('Invalid method');
end
addpath('functions/');

linewidth = 2;
marker_size = 10;

dt = 0.1; % [s]
sim_t = 100; % [s]
I = 1; % inertia
T = sim_t/dt; % number of iterations
t_vect = dt:dt:sim_t;
max_iter = 50;
max_line_search = 10;
V_z = -5; % [m/s]
V = 10; % [m/s] wind speed

x = zeros(4,T); % chute position
u = zeros(1,T); % input
nu = zeros(6,T); % non controllable input
initial_2D = [50 50 0];
initial_3D = [50 50 200 0];
x(:, 1) = initial_3D; % initial state
x_est = zeros(4,T);
x_est(:,1) = x(:,1);
P_est = zeros(4,4);
target_2D = [0,0,0]';
[n,~] = size(x);

% DDP init
x_bar = zeros(n-1,T);
u_bar = zeros(1,T);

% State covariance matrix
R_GPS = rand(4,4)-0.5;
R_GPS = R_GPS*R_GPS'; % bisogna cambiare l'incertezza di theta perche rad

% Input covariance matrix
Q = (rand(1,1)-0.5);
Q = Q*Q';

% Distrubances covariance matrix
L = 5*(rand(6,6)-0.5);
L = L*L';

A = eye(4,4);
B =[0;0;0;dt];
G = cell(1, T);
G_est = cell(1, T);

% Cost matrices
S = 5*eye(4);
R = 1;
Sf = 10*eye(4);

round = 0;

if strcmp(method, 'lqr')
    %% Pseudo-LQR
    
    % Initialization
    P = cell(1,T+1);
    P{T+1} = Sf;
    
    % Kalman Filter
    for t=1:T-1
        nu(:,t) = 0.1*randn(6,1);
        nu(1,:) = V;
        nu(5,:) = V_z;
        nu_unc(:,t) = nu(:,t) + mvnrnd([0;0;0;0;0;0], L)';  % noise on the non controllable inputs
        
        % LQR algorithm
        for i=T:-1:t
          P{i} = S+A'*P{i+1}*A-A'*P{i+1}*B*inv(R+B'*P{i+1}*B)*B'*P{i+1}*A;
        end

        % Angle of the chute wrt the x axis
        if t > 1
            if x_est(2,t) > 0 && x_est(2,t-1) < 0 && x_est(4,t) > 2*pi
                round =+ 1;
            end
        end

        if x_est(1,t) - target_2D(1) > 0 && x_est(2,t) - target_2D(2) > 0
            alpha(t) = atan2(x_est(2,t) - target_2D(2), x_est(1,t) - target_2D(1));          
        elseif x_est(1,t) - target_2D(1) > 0 && x_est(2,t) - target_2D(2) < 0
            alpha(t) = atan2(x_est(2,t) - target_2D(2), x_est(1,t) - target_2D(1)) + 2*pi;  
        elseif x_est(1,t) - target_2D(1) < 0 && x_est(2,t) - target_2D(2) > 0
            alpha(t) = atan2(x_est(2,t) - target_2D(2), x_est(1,t) - target_2D(1));      
        elseif x_est(1,t) - target_2D(1) < 0 && x_est(2,t) - target_2D(2) < 0
            alpha(t) = atan2(x_est(2,t) - target_2D(2), x_est(1,t) - target_2D(1)) + 2*pi;
        end

        % % Angle of the chute wrt the x axis
        % if x_est(1,t) - target_2D(1) > 0 && x_est(2,t) - target_2D(2) > 0
        %     alpha(t) = atan2(x_est(2,t) - target_2D(2), x_est(1,t) - target_2D(1));
        %     theta_des = alpha(t) + pi/2;          
        % elseif x_est(1,t) - target_2D(1) > 0 && x_est(2,t) - target_2D(2) < 0
        %     alpha(t) = atan2(x_est(2,t) - target_2D(2), x_est(1,t) - target_2D(1)) + pi/2;
        %     theta_des = -alpha(t) + 5*pi/2;  
        % elseif x_est(1,t) - target_2D(1) < 0 && x_est(2,t) - target_2D(2) > 0
        %     alpha(t) = atan2(x_est(2,t) - target_2D(2), x_est(1,t) - target_2D(1)) - pi/2;  
        %     theta_des = -alpha(t) + 3*pi/2;    
        % elseif x_est(1,t) - target_2D(1) < 0 && x_est(2,t) - target_2D(2) < 0
        %     alpha(t) = atan2(x_est(2,t) - target_2D(2), x_est(1,t) - target_2D(1)) + pi;
        %     theta_des = alpha(t) + 3*pi/2;
        % end

        % Desired angle of the chute wrt the x axis
        theta_des = alpha(t) + pi/2 + 2*pi*round;

        % Optimal input
        K = inv(R+B'*P{t+1}*B)*B'*P{t+1}*A;
        target = [0,0,0,theta_des]';
        u(:,t) = -K*(x_est(:,t)-target);
        u_unc(:,t) = u(:,t) +  mvnrnd(0, Q)'; % noise on the inputs
        
        % State Update
        G{t} = [-sin(x(4,t))*dt 1 0 0 0 0;
                cos(x(4,t))*dt 0 1 0 0 0;
                0 0 0 1 dt 0;
                0 0 0 0 0 1];
        x(:, t+1) = A*x(:,t)+B*u(:,t)+G{t}*nu(:,t);
        
        % Predictions
        G_est{t} = [-sin(x_est(4,t))*dt 1 0 0 0 0;
                cos(x_est(4,t))*dt 0 1 0 0 0;
                0 0 0 1 dt 0;
                0 0 0 0 0 1];
        x_est(:, t+1) = A*x_est(:,t)+B*u_unc(:,t)+G_est{t}*nu_unc(:,t); % non conosciamo nu se non lo misuriamo 
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
            break
        end
    end
elseif strcmp(method, 'ddp')

    %% DDP
    
    % Initialization
    % Cost matrices
    S = 0.1*eye(3);
    R = 0.01;
    Sf = 10000*eye(3);
    cost = zeros(1, max_iter);
    d1 = 0;
    d2 = 0;
    mu = 1e-4;
    min_cost_impr = 1e-5;
    alpha_factor = 0.5;
    mu_factor = 10;
    mu_max = 100;
    min_alpha_to_increase_mu = 0.05;
    exp_impr_th = 1e-3;
    converged = false;
    th = 1e-2;
    K = cell(1,T);
    for i=1:T
        K{i} = zeros(1,3);
    end

    for j=1:max_iter
        % Dynamic Simulation
        [x_bar, u_bar] = sim_dyn(x_bar, u_bar, n, T, initial_2D, K, dt, V);
        
        % Backward Pass
        [w_bar, K, Q_u, Q_uu] = backward_pass(x_bar,u_bar, mu, S, Sf, R, T, n, target_2D, dt, V);

        % Forward Pass
        alpha = 1;
        line_search = false;
        l_f = (x_bar(:,end)-target_2D)'*Sf*(x_bar(:,end)-target_2D);
        cost(1,j) = l_f;
        for p=1:T
            cost(1,j) = cost(1,j) + running_cost(p,x_bar,u_bar, S, R, target_2D);
        end
        for s=1:T-1
            d1 = d1 + w_bar(:,s)'*Q_u{s};
            d2 = d2 + 0.5*w_bar(:,s)'*Q_uu{s}*w_bar(:,s);
        end

        for jj=1:max_line_search

            [x_new,u_new]  = sim_dyn(x_bar, u_bar+alpha*w_bar, n, T, initial_2D, K, dt, V);
            new_cost = l_f;
            for p=1:T
                new_cost = new_cost + running_cost(p,x_new,u_new, S, R, target_2D);
            end
            exp_impr = alpha*d1+0.5*alpha^2*d2;
            if exp_impr > 0
                exp_impr = -1;
            end
            rel_impr = (new_cost-cost(1,j))/exp_impr;
            
            if rel_impr > min_cost_impr
                fprintf("---------------------------------------------------------------------------------------\n")
                fprintf("Cost improved from %.3f to %.3f. Exp. impr %.3f. Rel. impr. %.1f\n" , cost(1,j), new_cost, exp_impr, 1e2*rel_impr);
                line_search = true;
            end

            if line_search
                u_bar = u_bar + alpha*w_bar;
                cost(1,j+1) = new_cost;
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
                fprintf("Alpha is small: increasing mu to %f\n", mu);
                if mu>mu_max
                    fprintf('Max regularization reached. Algorithm failed to converge\n');
                    converged = true;
                end
            end
        end
        
        if ((abs(exp_impr) < exp_impr_th) || norm((target_2D - x_bar(:,end))) < th)
            fprintf("Algorothm converged. Expected improvment %f\n", exp_impr);
            converged = true;
        end

        if converged
            break
        end

   
    end

    [x_bar, u_bar] = sim_dyn(x_bar, u_bar, n, T, initial_2D, K, dt, V);
    
    % % Kalman Filter
    % for t=1:T-1
    %     nu(:,t) = 0.1*randn(6,1);
    %     nu(4,:) = V_z;
    %     nu_unc(:,t) = nu(:,t) + mvnrnd([0;0;0;0;0], L)';

    %     u_unc(:,t) = u_bar(:,t) +  mvnrnd([0;0;0], Q)'; % noise on the inputs

    %     % State Update
    %     B{t} = dt*[cos(x(4,t)), -sin(x(4,t)), 0;
    %         sin(x(4,t)), cos(x(4,t)), 0;
    %          0, 0, 0;
    %          0, 0, 1];
    %     x(:,t+1) = A*x(:,t) + B{t}*u_bar(:,t) + G*nu(:,t);
        
    %     % Linearization
    %     A_t = eye(4);
    %     A_t(:,4) = [-u_bar(1,i)*sin(x_bar(3,i)*dt-u_bar(2,i)*cos(x_bar(3,i))*dt);
    %                 u_bar(1,i)*cos(x_bar(3,i))*dt-u_bar(2,i)*sin(x_bar(3,i))*dt;
    %                 0;
    %                 1];
        
    %     % Predictions
    %     B_est{t} = dt*[cos(x_est(4,t)), -sin(x_est(4,t)), 0;
    %         sin(x_est(4,t)), cos(x_est(4,t)), 0;
    %          0, 0, 0;
    %          0, 0, 1];
    %     x_est(:,t+1) = A*x_est(:,t) + B_est{t}*u_unc(:,t) + G*nu_unc(:,t);
    %     P_est = A_t*P_est*A_t' + B{t}*Q*B{t}' + G*L*G';
    %     % Measurements update
    %     z_GPS = x(:,t+1) + mvnrnd([0;0;0;0], R_GPS)';
    %     Innovation = z_GPS - x_est(:,t+1);
    %     H_GPS = eye(4,4);
    %     % Updated Kalman estimates
    %     S_Inno = H_GPS*P_est*H_GPS' + R_GPS; % covariance of inno
    %     W = P_est*H_GPS'*inv(S_Inno); % kalman filter gain
    %     x_est(:,t+1) = x_est(:,t+1) + W*Innovation; % updated state estimate
    %     P_est = (eye(4) - W*H_GPS)*P_est; % updated cov matrix
        
    %     if x(3,t) < 0
    %        x(:,t:T) = zeros(4,T-t+1);
    %         break
    %     end
    %     u = u_bar;
    % end

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
plot(t_vect, V_z*ones(1,length(t_vect)), 'k--', 'LineWidth', linewidth)
plot(t_vect, V*ones(1,length(t_vect)), 'b--', 'LineWidth', linewidth)
grid on
legend('\omega','V_z','V', 'Location','best')

if strcmp(method, 'ddp')
    figure(6)
    plot(1:max_iter, cost(1,:))
end