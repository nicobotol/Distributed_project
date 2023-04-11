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
B = dt*[cos(x(3,t)), 0;
     sin(x(3,t)), 0;
     0, 1];
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

%% Linearization

cost = x'*Q*x + u'*R*u + x(:,end)'*Qf*x(:,end);

% Initialization
V_x = zeros(N,2);
V_xx = zeros(2,2);
V_u = zeros(N,2);
V_uu = zeros(2,2);
f_x = ;
f_u = ;

V_x(N,:) = x(:,end)'*(Qf+Qf');
V_xx(N,:) = (Qf+Qf');

for i=N-1:-1:0
    Qx = x(:,i)'*(Q+Q') + f_x'*V_x(i+1);
    Qu = u(:,i)*(R+R') + f_u'*V_u(i+1);
end



