function [j_fig, agents] = plot_chutes_time_evo(agents, true_centroid_store, t, par)
  
  n_agents = par.n_agents;
  target = par.target;
  colors_vect = par.colors_vect;
  line_width = par.line_width;
  marker_size = par.marker_size;
  x0 = par.x0;
  v_free_falling = par.v_free_falling;
  position_range = par.position_range;
  
  j_fig = 0;

  for i=1:n_agents
    agents{i}.P_print{t} = agents{i}.P_est;
  end

  j_fig = j_fig + 1;
  figure(j_fig);clf;
  hold all
  for i=1:n_agents
    % get the number of the color to be used
    i_color = 1 + mod(i-1, 7); 
    % Plot the actual agents position and its encumbrance
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
  xlim([-abs(1.5*position_range+x0(1)) abs(1.5*position_range+x0(1))])
  ylim([-abs(1.5*position_range+x0(2)) abs(1.5*position_range+x0(2))])
  zlim([0 1.5*position_range+x0(3)])
  title(['Iteration number: ' num2str(t)])
  grid on
  % axis equal
  view(45,45)
  drawnow

end