clear all;
close all;
clc;

%% LQR

% Initialization
p = [1;8];
dt = 0.01;
N = 100;
l = 1:N;
x = zeros(2,length(l));
x(:,1) = [0;0];
u = zeros(2,length(l));

A = eye(2,2);
Q = [10, 0;
    0, 10];
R = 0.01*eye(2,2);
P = cell(1,N);
Qf = [10, 0;
    0, 10];
P{N} = Qf;
B = dt*[1, 0;
     0, 1];

% LQR algorithm
for i=N:-1:2
    P{i-1} = Q+A'*P{i}*A-A'*P{i}*B*inv(R+B'*P{i}*B)*B'*P{i}*A;
end

for t=1:N-1
    K = inv(R+B'*P{t+1}*B)*B'*P{t+1}*A; % non usiamo P(1)?
    u(:,t) = -K*(x(:,t)-p);
    x(:,t+1) = A*x(:,t) + B*u(:,t);
    fprintf('---------------------------------------------------------\n')
    fprintf('Time instant %d\n', t)
    fprintf('Actual State: [%d, %d\n', x(1,t), x(2,t));
    fprintf('Desired State: [%d, %d]\n', p(1), p(2));
    fprintf('Error: %d\n', sqrt((x(1,t)-p(1))^2+(x(2,t)-p(2))^2));
end
%% Plots

figure(1), clf, hold on;
plot(p(1), p(2),'k.','MarkerSize',20);
plot(x(1,:), x(2,:));
xlim([0 10])
ylim([0 10])