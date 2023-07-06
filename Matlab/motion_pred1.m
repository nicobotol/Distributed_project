clc;
close all;
clear all;

%% 

% import functions folder
addpath(genpath('functions'))

x = [0 0];
y = [3 2];
theta = pi/4;

% Problem when collapsing the circle to a point (the cone becomes a line)
dy = norm([-sin(theta) cos(theta)]*(y - x)');

B = circle(y(1),y(2), dy);
% tr = triangulation(polyshape(B));
% model = createpde;
% tnodes = tr.Points';
% telements = tr.ConnectivityList';
% geometryFromMesh(model,tnodes,telements);
% meshed_B = generateMesh(model,"Hmin",0.001,"GeometricOrder","linear"); % generate the mesh

% My = convexhull(x, meshed_B);
% shape = alphaShape(My(1,:)', My(2,:)', 1.5);
% edges = boundaryFacets(shape);
z = [B; x];

hull = convhull(z(:,1), z(:,2));

%% Plots

figure(1)
plot(x(1), x(2), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'k')
hold on
plot(y(1), y(2), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'k')
plot(B(:,1), B(:,2), 'k')
% plot meshed area
% pdemesh(meshed_B)
% plot convex hull
% plot(My(1,:), My(2,:), '.r')
% plot(shape)
% plot(shape.Points(edges',1), shape.Points(edges',2), 'b')
plot(z(hull,1), z(hull,2), 'r')
axis equal
