clear all;
close all;
clc;

%% LQR Initialization

p = [-3;8;pi];
dt = 0.01;
N = 200;
MAX_ITER = 100;
l = 1:MAX_ITER;
x = zeros(3,length(l));
% I initiliaze all the state as initial position, so that in the first
% iteration the B matrix is constant
for i=1:N
    x(:,i) = [1;1;pi/2];
end
u = zeros(2,length(l));
error = zeros(1,MAX_ITER);

A = eye(3,3);
B = cell(1,N);
C = [1;1];
Q = [4, 0, 0;
    0, 10, 0;
    0, 0, 1];
R = 0.01*eye(2,2);
P = cell(1,N+1);
Qf = [5, 0, 0;
    0, 10, 0;
    0, 0, 1];
P{N+1} = Qf;
%% LQR algorithm
% the idea is that the matrix B is calculated taking in consideration the
% optimal trajectory calculated in the previous step. It's an eurustic
% method to pseudo-linearize a NL system. I can do it in a offline for
% cycle (using iter) or online (using t). The difference is that the online
% take into consideration the actual state, so it can provide for noises
% and distrubances. In both cases, I take the global optimum.
% An alternative (more correct) would be to use the DPP, but I can't use it
% online, since the B matrix would be just a constant.

TH = 1e-1;

% for t=1:N-1
for iter=1:MAX_ITER    % in this way, I calulate the trajectory offline
    for i=N:-1:iter
        B{i} = dt*[cos(x(3,i)), 0;
        sin(x(3,i)), 0;
         0, 1]; % B follows the simulated trajectory
        P{i} = Q+A'*P{i+1}*A-A'*P{i+1}*B{i}*inv(R+B{i}'*P{i+1}*B{i})*B{i}'*P{i+1}*A;
    end
    for j=iter:N
        K = inv(R+B{j}'*P{j+1}*B{j})*B{j}'*P{j+1}*A; % non usiamo P(1)?
        u(:,j) = -K*(x(:,j)-p);
        x(:,j+1) = A*x(:,j) + B{j}*u(:,j);
    end
    error(1,iter) = sqrt((x(1,iter)-p(1))^2+(x(2,iter)-p(2))^2);
    fprintf('---------------------------------------------------------\n')
    fprintf('Iteration %d\n', iter)
    fprintf('Actual State: [%d, %d, %d]\n', x(1,iter), x(2,iter), x(3,iter));
    fprintf('Desired State: [%d, %d, %d]\n', p(1), p(2), p(3));
    fprintf('Error: %d\n', error(1,iter));
    if error(1,iter) < TH
        break
    end
end

fprintf('Number of iterations: %d\n', iter);

%% Plots

figure(1), clf, hold on;
plot(p(1), p(2),'k.','MarkerSize',20);
plot(x(1,:), x(2,:),'LineWidth',1);
xlim([-10 10])
ylim([-10 10])

figure(2), clf, hold on;
plot(1:iter, error(1,1:iter),'r','LineWidth',1)
xlabel('Iteration')
ylabel('Error')

figure(3), clf, hold on;
plot(1:N, u(1,:),'b','LineWidth',1)
plot(1:N, u(2,:),'g','LineWidth',1)
legend('Velocity','Angular Velocity');
xlabel('Instant')
ylabel('Magnitude')