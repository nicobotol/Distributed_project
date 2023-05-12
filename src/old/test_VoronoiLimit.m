clear
close all
clc

n_agents = 7;                   % number of agents
p = 5*(rand(2, agents) - 0.5);  % coordinates of the agents
Rs = 1.5;                       % sensing range

%% External boundary
R = 5;                          % radius inside which all the agetns have to remain
external_circle =  circle(mean(p(1, :)), mean(p(2, :)), R);

%% Pure voronoi limits
[V,C, XY] = VoronoiLimit(p(1,:), p(2,:), 'bs_ext', external_circle);
[~, idx] = ismember(p(1, :), XY(:, 1)); % position to remap results of the tessellation in the order of the intial point

lim_intersect = cell(7, 1);
lim_voronoi = cell(7, 1); 
lim_circle_rs = cell(7, 1);
for i=1:n_agents
  lim_voronoi{i} = polyshape(V(C{idx(i)}, 1), V(C{idx(i)}, 2)); % pure voronoi cell
  circle_rs =  circle(p(1, i), p(2, i), Rs);
  lim_circle_rs{i} = polyshape(circle_rs(:, 1), circle_rs(:, 2));
  lim_intersect{i} = intersect(lim_voronoi{i}, lim_circle_rs{i});
end

figure();
for i=1:n_agents
  subplot(4, 2, i)
  plot(lim_intersect{i},'FaceColor','red','FaceAlpha',0.1)
  hold on 
  plot(lim_voronoi{i},'FaceColor','green','FaceAlpha',0.1)
  plot(lim_circle_rs{i}, 'LineStyle', '--')
  plot(p(1, i), p(2, i), 'x')
  axis equal
  area_cell(i) = area(pol);
end

figure()
voronoi(p(1,:), p(2, :));
hold on
plot(XY(:, 1), XY(:, 2), 'x')
axis equal