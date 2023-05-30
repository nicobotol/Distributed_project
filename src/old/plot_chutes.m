function [] = plot_chutes(agents)

parameters; % load the constant parameters

n_agents = length(agents);
j_fig = 0;

%% Plots of the mesh
% j_fig = j_fig+1;
% figure(j_fig);clf;
% for i=1:n_agents
%   plot3(agents{i}.element_centroid(1,:), agents{i}.element_centroid(2,:), agents{i}.phi, 'o')
%   hold on
% end
% grid on
% axis equal

%% Plot of the 2D projection
% compute the centralized global centroid
g_centroid = zeros(3,1);
for i = 1:n_agents
  g_centroid = g_centroid + agents{i}.x_real/n_agents;
end
j_fig = j_fig+1;
figure(j_fig); clf;
axis equal
hold on
for i=1:n_agents
  plot(agents{i}.x_real(1), agents{i}.x_real(2), 'xr', 'MarkerSize', 20); % agent real position
  encumbrance = circle(agents{i}.x(1, i), agents{i}.x(2, i), agents{i}.delta);
  plot(encumbrance(:,1), encumbrance(:,2), '--g', 'LineWidth', 1.5); % encumbrance
  plot(agents{i}.voronoi);  % voronoi cell
  plot(agents{i}.global_centroid(1), agents{i}.global_centroid(2), 'diamond', 'Color', 'k', 'MarkerSize', 10);  % global centroid estimated by the agent

end
plot(g_centroid(1), g_centroid(2), 'pentagram', 'LineWidth',5, 'MarkerSize',10)
legend("cell", "agent", "cell geom. cent.", "cell weigh cent.", "Encumbrance", "Location","eastoutside");

% Plot of the 3D agents distributions
j_fig = j_fig+1;
figure(j_fig); clf;
axis equal
hold on
% plot the real global centroids
plot3(g_centroid(1), g_centroid(2), g_centroid(3), 'pentagram', 'LineWidth',5, 'MarkerSize',2)
for i=1:n_agents
  % draw a cylinder with the agent's encumbrance
  [X,Y,Z] = cylinder(agents{i}.delta); 
  surf(X + agents{i}.x(1, i), Y + agents{i}.x(2, i), Z*agents{i}.z_th + agents{i}.x(3, i), 'FaceColor', 'g', 'FaceAlpha', 0.2, 'EdgeColor', 'none'); % plot a cylinder cylinder
  
  % plot agent real and estimated positions
  plot3(agents{i}.x_real(1), agents{i}.x_real(2), agents{i}.x_real(3), 'xr', 'MarkerSize', 20); 
  plot3(agents{i}.x(1, i), agents{i}.x(2, i), agents{i}.x(3, i), 'o', 'MarkerSize', 10, 'Color', 'g'); 

  % Plot the voronoi cell based on agent's position
  tmp_ones = ones(length(agents{i}.voronoi.Vertices(:,2)));
  plot3(agents{i}.voronoi.Vertices(:,1), agents{i}.voronoi.Vertices(:,2), agents{i}.x(3, i)*tmp_ones);

  % plot the real and estimated global centroids
  plot3(agents{i}.global_centroid(1), agents{i}.global_centroid(2), agents{i}.global_centroid(3), 'diamond', 'Color', 'k', 'MarkerSize', 10);  % global centroid estimated by the agent
end
legend("global cnt", "agent encumbrance", "real agent", "est. agent", "voronoi", "est. global cnt", "Location","eastoutside");

% Plot of the 3D agents distributions
j_fig = j_fig+1;
figure(j_fig); clf;
axis equal
hold on
% plot the real global centroids
plot3(g_centroid(1), g_centroid(2), g_centroid(3), 'pentagram', 'LineWidth',5, 'MarkerSize',2)
for i=1:n_agents
  % draw a sphere with agent's communication range
  [X,Y,Z] = sphere();
  surf(X*agents{i}.Rc + agents{i}.x_real(1), Y*agents{i}.Rc + agents{i}.x_real(2), Z*agents{i}.Rc + agents{i}.x_real(3), 'FaceColor', 'b', 'FaceAlpha', 0.05, 'EdgeColor', 'none'); % plot a sphere

  % draw a cylinder with the agent's encumbrance
  [X,Y,Z] = cylinder(agents{i}.delta); 
  surf(X + agents{i}.x(1, i), Y + agents{i}.x(2, i), Z*agents{i}.z_th + agents{i}.x(3, i), 'FaceColor', 'g', 'FaceAlpha', 0.2, 'EdgeColor', 'none'); % plot a cylinder cylinder
  
  % plot agent real and estimated positions
  plot3(agents{i}.x_real(1), agents{i}.x_real(2), agents{i}.x_real(3), 'xr', 'MarkerSize', 20); 
  plot3(agents{i}.x(1, i), agents{i}.x(2, i), agents{i}.x(3, i), 'o', 'MarkerSize', 10, 'Color', 'g'); 

  % Plot the voronoi cell based on agent's position
  tmp_ones = ones(length(agents{i}.voronoi.Vertices(:,2)));
  plot3(agents{i}.voronoi.Vertices(:,1), agents{i}.voronoi.Vertices(:,2), agents{i}.x(3, i)*tmp_ones);

  % plot the real and estimated global centroids
  plot3(agents{i}.global_centroid(1), agents{i}.global_centroid(2), agents{i}.global_centroid(3), 'diamond', 'Color', 'k', 'MarkerSize', 10);  % global centroid estimated by the agent
end
legend("global cnt", "agent encumbrance", "real agent", "est. agent", "voronoi", "est. global cnt", "Location","eastoutside");


end