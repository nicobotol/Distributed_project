clear all;
close all;
clc;

%% LQR

% Initialization
p = [3;4;0;0]; % if angle is zero -> not solvable
dt = 0.01;
N = 100;
l = 1:N;
x = zeros(4,length(l));
x(:,1) = [0;0;pi/2;0];
u = zeros(2,length(l));

Q = eye(4,4);
R = 0.01*eye(2,2);
P = cell(1,N);
Qf = Q;
P{N} = Qf;

% LQR algorithm
for t=1:N-1
    A = [1,0,0,dt*cos(x(3,t));
         0,1,0,dt*sin(x(3,t));
         0,0,1,0;
         0,0,0,1];
    B = [1/2*dt^2*cos(x(3,t)),0;
         1/2*dt^2*sin(x(3,t)),0;
         0,1/2*dt^2;
         dt,0];
    for i=N:-1:2
        P{i-1} = Q+A'*P{i}*A-A'*P{i}*B*inv(R+B'*P{i}*B)*B'*P{i}*A;
    end
    K = inv(R+B'*P{t+1}*B)*B'*P{t+1}*A; % non usiamo P(1)?
    u(:,t) = -K*(x(:,t)-p);
    x(:,t+1) = A*x(:,t) + B*u(:,t);
    fprintf('---------------------------------------------------------\n')
    fprintf('Time instant %d\n', t)
    fprintf('Actual State: [%d, %d, %d]\n', x(1,t), x(2,t), x(3,t));
    fprintf('Desired State: [%d, %d, %d]\n', p(1), p(2), p(3));
    fprintf('Error: %d\n', sqrt((x(1,t)-p(1))^2+(x(2,t)-p(2))^2));
end

%% Plots

figure(1), clf, hold on;
plot(p(1), p(2),'k.','MarkerSize',20);
plot(x(1,:), x(2,:));
xlim([0 10])
ylim([0 10])