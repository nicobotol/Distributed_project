clc;
close all;
clear all;

%% 

% import functions folder
addpath(genpath('functions'))
x = [0 0];
y = [0.6 0.3];
theta = pi/4;
alpha = 0.1;

figure(1)
plot(x(1), x(2), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'k')
hold on
plot(y(1), y(2), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'k')
for i=1:2

  % Problem when collapsing the circle to a point (the cone becomes a line)
  dy = norm([-sin(theta) cos(theta)]*(y - x)');

  % Circle centerd in y with radius equal to the goal alignment distance
  B = circle(y(1),y(2), dy);
  B_shape = polyshape(B(:,1), B(:,2));

  w = x + ([cos(theta) sin(theta)]'*[cos(theta) sin(theta)]*(y - x)')';

  % Points for the convex hull operator
  z = [x; y; w];

  hull = convhull(z(:,1), z(:,2));

  % Truncated cone
  shape = polyshape(z(hull,1), z(hull,2));
  cone = union(shape, B_shape);

  %% Plots

  % plot(B(:,1), B(:,2), 'k')
  plot(cone, 'FaceAlpha', alpha, 'FaceColor', 'r')
  % plot(z(hull,1), z(hull,2), 'r')
  % plot(cone.Vertices(:,1), cone.Vertices(:,2), 'b')
  xlim([-1 1])
  ylim([-1 1])

  y = [0.3 0.15];
  % x = x + [0.2 0.15];
  % theta = theta - pi/50;
  % alpha = alpha + 0.1;
end
hold off