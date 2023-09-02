clearvars;
close all;
clc
ag = cell(2,1);
ag{1}.Rs = 10;
ag{2}.Rs = 10;
ag{3}.Rs = 10;
ag{1}.Rc = 2*ag{1}.Rs;
ag{2}.Rc = 2*ag{2}.Rs;
ag{3}.Rc = 2*ag{3}.Rs;

ag{1}.x = [0 0; 0 6.1; 0 30]';
ag{2}.x = [0 0; 0 6.1; 0 30]';
ag{3}.x = [0 0; 0 6.1; 0 30]';

ag{1}.delta = 2;
ag{2}.delta = 2;
ag{3}.delta = 2;

ag{1}.vmaxdt = 15;
ag{2}.vmaxdt = 15;
ag{3}.vmaxdt = 15;

par.n_agents = 3;
par.epsilon = 1;
t=1;

ag = voronoi_chutes(ag, t, par);

figure()
hold on
for i=1:par.n_agents
  plot(ag{i}.voronoi);
  plot(ag{i}.x(1,i), ag{i}.x(2,i), 'x');
end
axis equal