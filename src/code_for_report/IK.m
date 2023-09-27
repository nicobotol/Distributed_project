clc;
clear all;
close all;

%% Data

ik = 'postural'; % 'centroid' or 'postural'

x1 = [1 0];
x2 = [0 1];
x3 = [1 1];

x1i = x1;
x2i = x2;
x3i = x3;

x = [x1;x2;x3];
w = 10e-3;
gamma = 10e-3;

target = [3 3];
target_x = [3 3];

centroid = sum(x)/3;
centroid_i = centroid;
u = 0.5*(target - centroid);
centroid_d = centroid + u;

x1_d = x1 + 0.5*(target_x - x1);
x2_d = x2 + 0.5*(target_x - x2);
x3_d = x3 + 0.5*(target_x - x3);

x1_same = x1 + u;
x2_same = x2 + u;
x3_same = x3 + u;

J = [1/3 0 1/3 0 1/3 0;
    0 1/3 0 1/3 0 1/3];
J_bar = [J;w*eye(6)];

e_c = [centroid_d - centroid]';
e_x = [[x1_d - x1]';
    [x2_d - x2]';
    [x3_d - x3]'];
e = [e_c;e_x];

% %% Iterative IK algorithm
% while norm(J'*e_c)^2 >= 0.01
%   alpha = 1;
%   delta_x = inv(J'*J + w^2*eye(6))*J'*e_c;
%   delta_x = reshape(delta_x,2,3)';
%   x_test = x + alpha*delta_x;
%   centroid = sum(x_test)/4;
%   e_test = [centroid_d - centroid]';
%   red_err = norm(e_c) - norm(e_test);
%   while red_err <= gamma*alpha*norm(e_c)
%     alpha = alpha/2;
%     x_test = x + alpha*delta_x;
%     centroid = sum(x_test)/4;
%     e_test = [centroid_d - centroid]';
%     red_err = norm(e_c) - norm(e_test);
%   end
%   x = x_test;
%   e_c = e_test;
% end

%% Iterative IK algorithm with postural task
while norm(e_c)^2 >= 0.01
  alpha = 1;
  delta_x = inv(J'*J + w^2*eye(6))*(J'*e_c + w^2*e_x);
  delta_x = reshape(delta_x,2,3)';
  x_test = x + alpha*delta_x;
  centroid = sum(x_test)/3;
  e_c_test = [centroid_d - centroid]';
  e_x_test = [[x1_d - x_test(1,:)]';
    [x2_d - x_test(2,:)]';
    [x3_d - x_test(3,:)]'];
  e_test = [e_c_test;e_x_test];
  red_err = norm(e) - norm(e_test);
  while red_err <= gamma*alpha*norm(e)
    alpha = alpha/2;
    x_test = x + alpha*delta_x;
    centroid = sum(x_test)/3;
    e_c_test = [centroid_d - centroid]';
    e_x_test = [[x1_d - x_test(1,:)]';
    [x2_d - x_test(2,:)]';
    [x3_d - x_test(3,:)]'];
    e_test = [e_c_test;e_x_test];
    red_err = norm(e) - norm(e_test);
  end
  x = x_test;
  e = e_test;
  e_c = [centroid_d - centroid]';
  e_x = [[x1_d - x(1)]';
    [x2_d - x(2)]';
    [x3_d - x(3)]'];
  J_bar = [J;w*eye(6)];
end

%% Same input


%% Plotting
fig = figure('Color','w');
plot(x1i(1),x1i(2),'bo','MarkerSize',20,'LineWidth',2,'DisplayName','Initial Position')
hold on
plot(x2i(1),x2i(2),'bo','MarkerSize',20,'LineWidth',2,'HandleVisibility','off')
plot(x3i(1),x3i(2),'bo','MarkerSize',20,'LineWidth',2,'HandleVisibility','off')
plot(x(1,1),x(1,2),'ro','MarkerSize',20,'LineWidth',2,'DisplayName','Final IK Position')
plot(x(2,1),x(2,2),'ro','MarkerSize',20,'LineWidth',2,'HandleVisibility','off')
plot(x(3,1),x(3,2),'ro','MarkerSize',20,'LineWidth',2,'HandleVisibility','off')
% if strcmp(ik, 'postural')
%   plot(x1_d(1),x1_d(2),'r.','MarkerSize',10,'LineWidth',2)
%   plot(x2_d(1),x2_d(2),'r.','MarkerSize',10,'LineWidth',2)
%   plot(x3_d(1),x3_d(2),'r.','MarkerSize',10,'LineWidth',2)
% end
plot(x1_same(1),x1_same(2),'o','Color',[0.4660 0.6740 0.1880],'MarkerSize',20,'LineWidth',2,'DisplayName','Final Position')
plot(x2_same(1),x2_same(2),'o','Color',[0.4660 0.6740 0.1880],'MarkerSize',20,'LineWidth',2,'HandleVisibility','off')
plot(x3_same(1),x3_same(2),'o','Color',[0.4660 0.6740 0.1880],'MarkerSize',20,'LineWidth',2,'HandleVisibility','off')
plot(centroid_i(1),centroid_i(2),'b.','MarkerSize',50,'LineWidth',2,'DisplayName','Initial Centroid')
plot(centroid(1),centroid(2),'r.','MarkerSize',70,'LineWidth',2,'DisplayName','Final Centroid IK')
plot(centroid(1),centroid(2),'.','Color',[0.4660 0.6740 0.1880],'MarkerSize',50,'LineWidth',2,'DisplayName','Final Centroid')
plot(centroid_d(1),centroid_d(2),'kx','MarkerSize',20,'LineWidth',2,'DisplayName','Desired Centroid')
legend('FontSize',20,'Location','southeast')
xlabel('X Position (m)')
ylabel('Y Position (m)')
grid on
title('High-level control comparison')
xlim([-0.5 3])
ylim([-0.5 2.3])
set(gca, 'FontSize', 30)
axis equal
export_fig(fig, 'high-level control comparison.eps');