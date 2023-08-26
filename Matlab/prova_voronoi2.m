%% voronoiPolyhedrons
clc;
clear all;
close all;

seeds = [0 1 3 4; 0 0 0 0];
lb = [-10 -10];
up = [10 10];
[A,B,vert] =  voronoiPolyhedrons(seeds, lb, up);

% compute voronoi cells
figure(); hold on;
for i=1:size(seeds,2)
  k = convhull(vert{i}(1,:),vert{i}(2,:));
  poly = polyshape(vert{i}(1,k), vert{i}(2,k));
  plot(poly);
  plot(seeds(1,i), seeds(2,i), 'k*', 'MarkerSize', 10);
end