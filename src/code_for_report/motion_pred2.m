clc;
close all;
clear all;
% import functions folder
if ispc
  path = '..\functions\';
else
  path = '../functions/';
end

addpath(path);
set(0,'DefaultFigureWindowStyle','docked')

%% Initialization
x = [0 0];      % actual point
y = [0.6 0.3];  % target point
x_in = x;
y_in = y;
theta = pi/3;   % actual orientation 
alpha = 0.1;    % cone transparency
R = sqrt(0.6*0.6 +0.5*0.5)+0.01;          % varonoi cell radius
omega_max = 5;
K_v = 1;        % linear velocity gain
K_omega = omega_max/(2*pi); % angular velocity gain
V = 80/3.6;          % max linear velocity
T = 1e2;        % simulation time
dt = 0.5;       % time step

figure(1)
hold on;
for i=1:10

  [cone, len_cone, dy] = feedback_motion_prediction_chute(theta, x, y);
  voronoi_cell_points = circle(x(1), x(2), R);
  voronoi_cell = polyshape(voronoi_cell_points(:,1), voronoi_cell_points(:,2));

  % Do different plots in case of polyshape or line
  if len_cone > 2 % if cone is a polyshape
    plot(cone, 'FaceAlpha', alpha, 'FaceColor', 'r')
  else
    plot(cone(:,1), cone(:,2), 'r-'); % if cone is a line
  end
  % plot(voronoi_cell, 'FaceAlpha', alpha, 'FaceColor', 'b')

  xlim([-0.8 0.8])
  ylim([-0.8 0.8])
  axis equal

  % y = [0.3 0.15];
  x = x + [0.05 0.04];
  theta = theta - pi/50;
  alpha = alpha + 0.001;
end
plot(x_in(1), x_in(2), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'k')
plot(y_in(1), y_in(2), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'k')
hold off

%% Check intersect
x = [0 0];      % actual point
y = [0.6 0.3];  % target point
theta = pi/3;   % actual orientation 
alpha = 0.1;    % cone transparency
[cone, len_cone, dy] = feedback_motion_prediction_chute(theta, x, y);
voronoi_cell_points = circle(x(1), x(2), R);
voronoi_cell = polyshape(voronoi_cell_points(:,1), voronoi_cell_points(:,2));

figure(2);hold on;
% Do different plots in case of polyshape or line
if len_cone > 2 % if cone is a polyshape
  plot(cone, 'FaceAlpha', alpha, 'FaceColor', 'k')
else
  plot(cone(:,1), cone(:,2), 'r-'); % if cone is a line
end
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

plot(voronoi_cell, 'FaceAlpha', alpha, 'FaceColor', 'y')
while (inside == 0)
  % If the cone goes outside the voronoi cell, then move the target point closer to the starting one of a quanty equal to how much the cone goes outside
  % [~, moving_radius] = incircle(out_area.Vertices(:,1), out_area.Vertices(:,2)); % radius of the motion of the point
  delta = p_poly_dist(y(1), y(2), voronoi_cell.Vertices(:,1), voronoi_cell.Vertices(:,2)); % radius of the motion of the point
  delta = abs(delta);
  theta2 = atan2(y(2) - x(2), y(1) - x(1)); % direction between the target point and the agent position
  moving_radius = min(delta, dy - delta);
  y = y - moving_radius*[cos(theta2) sin(theta2)]; % new target point
  [cone, len_cone, dy] = feedback_motion_prediction_chute(theta, x, y); % new cone

  intersec_area = intersect(voronoi_cell, cone);
  out_area = subtract(cone, voronoi_cell);

  len_intersec = size(intersec_area.Vertices, 1);
  if len_cone ~= len_intersec % cone outside voronoi
    inside = 0;
  else
    inside = 1;
  end
  plot(cone, 'FaceAlpha', alpha, 'FaceColor', 'k')
  plot(y(1), y(2), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'k')
end



xlim([-1 1])
ylim([-1 1])
axis equal

%% Simulation
x_store = x';
theta_store = theta;
for t=1:T
  % Optimal control
  u(1,t) = K_v*min(V, [cos(theta) sin(theta)]*(y - x)'); % forward velocity
  u(2,t) = K_omega*atan2([-sin(theta) cos(theta)]*(y - x)',[cos(theta) sin(theta)]*(y - x)'); % angular velocity

  % Update state
  x = x + u(1,t)*dt*[cos(theta) sin(theta)];
  theta = theta + u(2,t)*dt;

  x_store = [x_store x'];
  theta_store = [theta_store; theta];
end

p=plot(x_store(1, :), x_store(2, :), 'k-o', 'LineWidth', 2);
p.MarkerFaceColor = [1 0 0];
p.MarkerEdgeColor = [1 0 0];

% plot the controls
figure()
plot(u(1, :))
hold on
plot(u(2, :))
legend('v', '\omega')
