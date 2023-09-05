clearvars;
close all;
clc
ag = cell(2,1);
ag{1}.Rs = 10;
ag{2}.Rs = 10;
ag{1}.Rsv = 10;
ag{2}.Rsv = 10;
% ag{3}.Rs = 10;
ag{1}.Rc = 2*ag{1}.Rs;
ag{2}.Rc = 2*ag{2}.Rs;
ag{1}.Rcv = 2*ag{1}.Rsv;
ag{2}.Rcv = 2*ag{2}.Rsv;
% ag{3}.Rc = 2*ag{3}.Rs;

ag{1}.x = [0 0 0; 0 6.01 22]';
ag{2}.x = ag{1}.x;
% ag{3}.x = [0 0; 0 6; 0 30]';

ag{1}.z_min = [];
ag{2}.z_min = [];
ag{1}.z_min_old = 0;
ag{2}.z_min_old = 0;

ag{1}.delta = 2;
ag{2}.delta = 2;
ag{2}.z_th = 2;
ag{1}.z_th = 2;

ag{1}.vmaxdt = 10;
ag{2}.vmaxdt = 10;
% ag{3}.vmaxdt = 15;

ag{1}.P_est = cell(2,1);
ag{2}.P_est = cell(2,1);
ag{1}.P_est{1} = 1*eye(3);
ag{1}.P_est{2} = 1*eye(3);
ag{2}.P_est{1} = 1*eye(3);
ag{2}.P_est{2} = 1*eye(3);

par.n_agents = 2;
par.epsilon = 1e-5;
par.coverage = 1;
t=1;

ag = voronoi_chutes(ag, t, par);

figure()
hold on
for i=1:par.n_agents
  plot(ag{i}.voronoi);
  plot(ag{i}.x(1,i), ag{i}.x(2,i), 'x');
end
% plot(ag{1}.agents_x_voronoi(1, 1), ag{1}.agents_x_voronoi(2,1), 'o')
% plot(ag{2}.agents_x_voronoi(1, 1), ag{2}.agents_x_voronoi(2,1), '*')
axis equal
grid on