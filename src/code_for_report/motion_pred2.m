clc;
close all;
clear all;
% import functions folder
addpath(genpath('functions'))

%% Initialization
x = [0 0];      % actual point
y = [0.6 0.3];  % target point
theta = pi/4;   % actual orientation 
alpha = 0.1;    % cone transparency
R = sqrt(0.6*0.6 +0.5*0.5)+0.01;          % varonoi cell radius

figure(1)
plot(x(1), x(2), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'k')
hold on
plot(y(1), y(2), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'k')
for i=1:2

  [cone, len_cone] = feedback_motion_prediction_chute(theta, x, y);
  voronoi_cell_points = circle(x(1), x(2), R);
  voronoi_cell = polyshape(voronoi_cell_points(:,1), voronoi_cell_points(:,2));

  % Do different plots in case of polyshape or line
  if len_cone > 2 % if cone is a polyshape
    plot(cone, 'FaceAlpha', alpha, 'FaceColor', 'r')
  else
    plot(cone(:,1), cone(:,2), 'r-'); % if cone is a line
  end
  plot(voronoi_cell, 'FaceAlpha', alpha, 'FaceColor', 'b')

  xlim([-1 1])
  ylim([-1 1])
  axis equal

  % y = [0.3 0.15];
  % x = x + [0.2 0.15];
  % theta = theta - pi/50;
  % alpha = alpha + 0.1;
end

%% Check intersect
intersec_area = intersect(voronoi_cell, cone);
out_area = subtract(cone, voronoi_cell);

if len_cone ~= -1 % if cone is a polyshape
  len_intersec = size(intersec_area.Vertices, 1);
  if len_cone ~= len_intersec % cone outside voronoi
    inside = 0;
  else
    inside = 1;
  end
else              % if cone is a segment
  inside = 1;
end

fprintf('Inside: %d\n', inside);


if inside == 0
  % If the cone goes outside the voronoi cell, then move the target point closer to the starting one of a quanty equal to how much the cone goes outside
  [C, delta] = incircle(out_area.Vertices(:,1), out_area.Vertices(:,2)); % radius of the motion of the point
  theta2 = atan2(y(2) - x(2), y(1) - x(1)); % direction between the target point and the agent position
  y2 = y - 2*delta*[cos(theta2) sin(theta2)]; % new target point
  [cone, len_cone] = feedback_motion_prediction_chute(theta, x, y2); % new cone

  plot(cone, 'FaceAlpha', alpha, 'FaceColor', 'r')
  plot(voronoi_cell, 'FaceAlpha', alpha, 'FaceColor', 'b')
  plot(y2(1), y2(2), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'k')
  circ = circle(C(1), C(2), delta);
  plot(circ(:,1), circ(:,2));

  xlim([-1 1])
  ylim([-1 1])
  axis equal
  hold off
end
