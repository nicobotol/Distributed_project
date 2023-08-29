function [j_fig, agents] = plot_chutes_time_evo(agents, true_centroid_store, t, par)
  
n_agents = par.n_agents;
n = par.n;
target = par.target;
colors_vect = par.colors_vect;
line_width = par.line_width;
marker_size = par.marker_size;

j_fig = 0;

for i=1:n_agents
  agents{i}.P_print{t} = agents{i}.P_est;
end

j_fig = j_fig + 1;
figure(j_fig);clf;
hold all
for i=1:n
  % get the number of the color to be used
  i_color = 1 + mod(i-1, 7); 
  % Plot the actual agents position and its encumbrance
  % encumbrance = circle(agents{i}.x_real(1), agents{i}.x_real(2), agents{i}.delta);
  % plot3(encumbrance(:, 1), encumbrance(:, 2), agents{i}.x_real(3)*ones(length(encumbrance(:,1))), '--g', 'Color', colors_vect(i_color, :))
  % plot3(encumbrance(:, 1), encumbrance(:, 2), agents{i}.x_real(3)*ones(length(encumbrance(:,1))) + agents{i}.z_th, '--g', 'Color', colors_vect(i_color, :)); % heigh of the incumbrance
  [X, Y, Z] = cylinder(agents{i}.delta, 4);
  Z = Z*agents{i}.z_th;
  surf(X+agents{i}.x_real(1), Y+agents{i}.x_real(2), Z+agents{i}.x_real(3), 'FaceColor', colors_vect(i_color, :), 'FaceAlpha', 0.5, 'EdgeColor', 'none');
  plot3(agents{i}.x(1,i), agents{i}.x(2,i), agents{i}.x(3,i),'x', 'Color', colors_vect(i_color, :))

  % Plot the estimated global centroid
  plot3(agents{i}.global_centroid(1), agents{i}.global_centroid(2), agents{i}.global_centroid(3),'diamond')

  % Plot the voronoi cell based on agent's position
  tmp_ones = ones(length(agents{i}.voronoi.Vertices(:,2)));
  plot3(agents{i}.voronoi.Vertices(:,1), agents{i}.voronoi.Vertices(:,2), agents{i}.x(3, i)*tmp_ones, 'Color', colors_vect(i_color, :), 'LineWidth', line_width);

  % Plot the motion prediction of the agent
%   tmp_ones = ones(length(agents{i}.motion_predict.Vertices(:,2)));
%   plot3(agents{i}.motion_predict.Vertices(:,1), agents{i}.motion_predict.Vertices(:,2), agents{i}.x(3, i)*tmp_ones, 'Color', colors_vect(i_color, :));
  
  % print the identifier
  text(agents{i}.x_real(1), agents{i}.x_real(2), agents{i}.x_real(3), num2str(i)) 
end
% Plot the starting point
plot3(agents{i}.x_store(1, 1), agents{i}.x_store(2,1), agents{i}.x_store(3, 1), 'x', 'MarkerSize', marker_size);
plot3(target(1), target(2), target(3), 'o', 'MarkerSize', marker_size,'DisplayName', 'TARGET');
% Plot the true global centroid
plot3(true_centroid_store(1, end), true_centroid_store(2, end), true_centroid_store(3, end), 'Marker','Pentagram', 'MarkerSize', marker_size)
xlabel('x [m] ')
ylabel('y [m]')
zlabel('z [m]')
xlim([-250 250])
ylim([-250 250])
zlim([0 550])
title(['Iteration number: ' num2str(t)])
grid on
% axis equal
view(45,45)
drawnow

% j_fig = j_fig + 1;
% figure(j_fig);clf;
% hold all
% for i=1:n
%   i_color = 1 + mod(i-1, 7);
%   % Plot the actual agents position and its encumbrance
%   encumbrance = circle(agents{i}.x_real(1), agents{i}.x_real(2), agents{i}.delta);
%   plot(encumbrance(:, 1), encumbrance(:, 2), '--g', 'Color', colors_vect(i_color, :))
%   plot(agents{i}.x_real(1), agents{i}.x_real(2),'x', 'Color', colors_vect(i_color, :));
%   plot(agents{i}.x(1, i), agents{i}.x(2, i),'^', 'Color', colors_vect(i_color, :));
%   plot(agents{i}.centroid(1), agents{i}.centroid(2),'o', 'Color', colors_vect(i_color, :));
%   % Plot the actual global centroid
%   plot(agents{i}.global_centroid(1), agents{i}.global_centroid(2),'diamond')

%   % Plot the voronoi cell based on agent's position
%   tmp_ones = ones(length(agents{i}.voronoi.Vertices(:,2)));
%   plot(agents{i}.voronoi.Vertices(:,1), agents{i}.voronoi.Vertices(:,2), 'Color', colors_vect(i_color, :));
%   text(agents{i}.x_real(1), agents{i}.x_real(2), num2str(i))
% end
% % Plot the starting point
% plot(agents{i}.x_store(1, 1), agents{i}.x_store(2,1), 'x', 'MarkerSize', marker_size);
% plot(target(1), target(2), 'o', 'MarkerSize', marker_size,'DisplayName', 'TARGET');
% plot(true_centroid_store(1, end), true_centroid_store(2, end), 'Marker','Pentagram', 'MarkerSize', marker_size)
% xlabel('x [m] ')
% ylabel('y [m]')
% lim = 60;
% xlim([-lim lim])
% ylim([-lim lim])
% grid on
% axis equal
% drawnow

% j_fig = j_fig + 1;
% figure(j_fig);
% hold all
% for i=1:n
%   i_color = 1 + mod(i-1, n_agents-1);
% 
%   % Plot the actual global centroid
%   plot(t, agents{i}.global_centroid(3),'diamond')
% 
% end
% xlabel('Time step [-]')
% ylabel('z [m]')
% title('Vertical position of the global centroid')
% grid on
% axis equal
% drawnow
% 
% j_fig = j_fig + 1;
% figure(j_fig);
% hold all
% for i=1:n
%   % Norm of the covariance matrix of the i-th agent
%   plot(t, norm(agents{i}.P_est{i}, 2),'diamond')
% 
% end
% xlabel('Time step [-]')
% ylabel('|P| [-]') 
% grid on
% axis equal
% drawnow

end