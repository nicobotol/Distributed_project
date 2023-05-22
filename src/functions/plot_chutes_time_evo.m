function [] = plot_chutes_time_evo(agents, true_centroid_store)

parameters; % load the constant parameters

n_agents = length(agents);
figure(1);clf;
hold all
for i=1:n
  i_color = 1 + mod(i-1, n_agents-1);
  % Plot the actual agents position and its encumbrance
  encumbrance = circle(agents{i}.x_real(1), agents{i}.x_real(2), agents{i}.delta);
  plot3(encumbrance(:, 1), encumbrance(:, 2), agents{i}.x_real(3)*ones(length(encumbrance(:,1))), '--g', 'Color', colors_vect(i_color, :))
  plot3(agents{i}.x_real(1), agents{i}.x_real(2), agents{i}.x_real(3),'x', 'Color', colors_vect(i_color, :))

  % Plot the estimated global centroid
  plot3(agents{i}.global_centroid(1), agents{i}.global_centroid(2), agents{i}.global_centroid(3),'diamond')

  % Plot the voronoi cell based on agent's position
  tmp_ones = ones(length(agents{i}.voronoi.Vertices(:,2)));
  plot3(agents{i}.voronoi.Vertices(:,1), agents{i}.voronoi.Vertices(:,2), agents{i}.x(3, i)*tmp_ones, 'Color', colors_vect(i_color, :));
end
% Plot the starting point
plot3(agents{i}.x_store(1, 1), agents{i}.x_store(2,1), agents{i}.x_store(3, 1), 'x', 'MarkerSize', marker_size);
plot3(target(1), target(2), target(3), 'o', 'MarkerSize', marker_size,'DisplayName', 'TARGET');
% Plot the true global centroid
plot3(true_centroid_store(1, end), true_centroid_store(2, end), true_centroid_store(3, end), 'Marker','Pentagram', 'MarkerSize', marker_size)
xlabel('x [m] ')
ylabel('y [m]')
zlabel('z [m]')
grid on
axis equal
view(45,45)
drawnow


figure(2);clf;
hold all
for i=1:n
  i_color = 1 + mod(i-1, n_agents-1);
  % Plot the actual agents position and its encumbrance
  encumbrance = circle(agents{i}.x_real(1), agents{i}.x_real(2), agents{i}.delta);
  plot(encumbrance(:, 1), encumbrance(:, 2), '--g', 'Color', colors_vect(i_color, :))
  plot(agents{i}.x_real(1), agents{i}.x_real(2),'x', 'Color', colors_vect(i_color, :))

  % Plot the actual global centroid
  plot(agents{i}.global_centroid(1), agents{i}.global_centroid(2),'diamond')

  % Plot the voronoi cell based on agent's position
  tmp_ones = ones(length(agents{i}.voronoi.Vertices(:,2)));
  plot(agents{i}.voronoi.Vertices(:,1), agents{i}.voronoi.Vertices(:,2), 'Color', colors_vect(i_color, :));
end
% Plot the starting point
plot(agents{i}.x_store(1, 1), agents{i}.x_store(2,1), 'x', 'MarkerSize', marker_size);
plot(target(1), target(2), 'o', 'MarkerSize', marker_size,'DisplayName', 'TARGET');
plot(true_centroid_store(1, end), true_centroid_store(2, end), 'Marker','Pentagram', 'MarkerSize', marker_size)
xlabel('x [m] ')
ylabel('y [m]')
grid on
axis equal
drawnow

end