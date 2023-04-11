clear all;
close all;
clc;

%% LQR

% Initialization
p = [-3;8;0]; % if angle is zero -> not solvable
dt = 0.01;
N = 100;
l = 1:N;
x = zeros(3,length(l));
x(:,1) = [1;1;pi/2];
u = zeros(2,length(l));

A = eye(3,3);
C = [1;1];
Q = [1, 0, 0;
    0, 5, 0;
    0, 0, 1];
R = 0.01*eye(2,2);
P = cell(1,N);
Qf = [5, 0, 0;
    0, 3, 0;
    0, 0, 1];
P{N} = Qf;

% LQR algorithm
for t=1:N-1
    B = dt*[cos(x(3,t)), 0;
     sin(x(3,t)), 0;
     0, 1];
    for i=N:-1:t+1
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
plot(x(1,:), x(2,:),'LineWidth',1);
xlim([-10 10])
ylim([-10 10])