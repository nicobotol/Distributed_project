function [] = plot_chutes(agents)

parameters; % load the constant parameters

n_agents = length(agents);
j_fig = 0;

j_fig = j_fig+1;
figure(j_fig);clf;
for i=1:n_agents
  plot3(agents{i}.element_centroid(1,:), agents{i}.element_centroid(2,:), agents{i}.phi, 'o')
  hold on
end
grid on
axis equal

% Plot the figure
j_fig = j_fig+1;
figure(j_fig); clf;
axis equal
for i=1:n_agents
  points = circle(agents{i}.x(1), agents{i}.x(2), agents{i}.delta);
  plot(agents{i}.voronoi);
  hold on
  plot(agents{i}.x(1), agents{i}.x(2), 'xr', 'MarkerSize', 20);
  plot(agents{i}.centroid_geometric(1), agents{i}.centroid_geometric(2), 'ob', 'MarkerSize', 10);
  plot(agents{i}.centroid(1), agents{i}.centroid(2), '*b', 'MarkerSize', 10);
  plot(points(:,1), points(:,2), '--g', 'LineWidth', 1.5); % encumbrance
  text(agents{i}.x(1), agents{i}.x(2), num2str(i), 'FontSize', 10);
end
plot(target(1), target(2), 'square', 'LineWidth',5, 'MarkerSize',10)
legend("cell", "agent", "geometric centroid", "weighted centroid", "Target", "Location","eastoutside");
% voronoi(global_positions(:,1), global_positions(:,2))

% Plot the figure
  j_fig = j_fig+1;
  figure(j_fig); clf;
  axis equal
  hold on
  for i=1:n_agents
    points_c = circle(agents{i}.x(1), agents{i}.x(2), Delta/2);
    tmp_ones = ones(length(agents{i}.voronoi.Vertices(:,2)));
    points_s = circle(agents{i}.x(1), agents{i}.x(2), agents{i}.Rs);
    c_ones = ones(length(points_c),1);
    s_ones = ones(length(points_s),1);
    plot3(agents{i}.voronoi.Vertices(:,1), agents{i}.voronoi.Vertices(:,2), agents{i}.x(3)*tmp_ones);
    plot3(agents{i}.x(1), agents{i}.x(2), agents{i}.x(3), 'xr', 'MarkerSize', 20);
    plot3(agents{i}.centroid_geometric(1), agents{i}.centroid_geometric(2), agents{i}.x(3), 'ob', 'MarkerSize', 10);
    plot3(agents{i}.centroid(1), agents{i}.centroid(2), agents{i}.x(3), '*b', 'MarkerSize', 10);
    plot3(points_c(:,1), points_c(:,2), agents{i}.x(3)*c_ones, '--g', 'LineWidth', 1.5);
  %   plot3(points_s(:,1), points_s(:,2), agents{i}.x(3)*c_ones, '--r', 'LineWidth', 1.5);
    % text(agents{i}.x(1), agents{i}.x(2), num2str(i), 'FontSize', 10);
  end
  legend("cell", "agent", "geometric centroid", "weighted centroid", "Target", "Location","eastoutside");

end