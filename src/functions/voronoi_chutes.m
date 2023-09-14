function [agents, delta_final] = voronoi_chutes(agents, t, par)
%% This functon computes the Voronoi cell for each agent, and furthermore it computes the dimension of the agent after having taken into account the uncertainty in the self localization

n_agents = par.n_agents;
coverage = par.coverage;
epsilon = par.epsilon;

for i = 1:n_agents
  % Initialization of the variables
  agents{i}.agents_x_voronoi = [];
  agents{i}.x_idx = [];
  agents{i}.z_min = min(agents{i}.x(3,i) - agents{i}.Rsv + coverage*sqrt(agents{i}.P_est{i}(3,3)), agents{i}.x(3,i));
  agents{i}.z_min_old = agents{i}.z_min;

  % Increase the encumbrabce of the agent in order to take into account the uncertainty on the knowledge of its own position
  agents_delta = agents{i}.delta;                                                % physical dimension of the agent i
  my_unct = max(sqrt(agents{i}.P_est{i}(1, 1)), sqrt(agents{i}.P_est{i}(2,2)));  % uncertainty on myself
  agents{i}.delta = agents{i}.delta + coverage*my_unct;                          % increase the encumbrance of the agent

  % Estimation of the positions of the other robots
  for j = 1:n_agents
   if j ~= i && ismember(j, agents{i}.visited_chutes)
    % The distance is measured twice: the first one is used to determine if the agent has to be taken inot account or not, while the second is used to determine the distance after having moved the agent j for taking into account the uncertainty on j
    dist = norm(agents{i}.x(1:2, i) - agents{i}.x(1:2, j));  % distance between robots in 2D plane
    dir = (agents{i}.x(1:2, i) - agents{i}.x(1:2, j))/dist;  % direction between i and j

    old_j_pos = agents{i}.x(1:2, j); % save the old position of agent j
    unc_j = max(sqrt(agents{i}.P_est{j}(1, 1)), sqrt(agents{i}.P_est{j}(2,2))); % uncertainty in the plane
    % Move the agent j closer to i in order to take into account the uncertainty on the position of j
    agents{i}.x(1:2, j) = agents{i}.x(1:2, j) + min(dist - epsilon, coverage*unc_j)*dir;
    dist = norm(agents{i}.x(1:2, i) - agents{i}.x(1:2, j));  % recompute distance to take into account that we have moved the agent j

    dist_z = agents{i}.x(3, i) - agents{i}.x(3, j);
    dist_z_norm = abs(dist_z); % distance between 2 robots in the vertical direction
    dir_z = dist_z/dist_z_norm; % direction between i and j in the vertical direction
    unc_z = coverage*(sqrt(agents{i}.P_est{i}(3,3)) + sqrt(agents{i}.P_est{j}(3,3))); % uncertainty in the z direction

    % Voronoi in z direction
    % Set the voronoi limit in the vertical direction below the agent. Each agent, once sees another one reasonably close to it sets the limit of the voronoi cell in the vertical direction below it. Initially the limit is the sensing range, but then it is moved closer to the agent in order to consider the uncertainty on the position and the velocity of the two 
    if dist_z_norm <= agents{i}.Rcv + unc_z + dir_z*(agents{i}.vmaxzdt - agents{j}.vmaxzdt) && dist_z >= 0 && dist <= agents{i}.Rc + coverage*(unc_j + my_unct)
      % set that in any case the limit cannot go above the agent in order to avoid negative control inputs
      agents{i}.z_min = min(agents{i}.x(3,i) - (dist_z_norm - agents{j}.z_th - coverage*unc_z), agents{i}.x(3,i)); 
    else
      agents{i}.z_min = min(agents{i}.x(3, i) - agents{i}.Rsv + coverage*sqrt(agents{i}.P_est{i}(3,3)), agents{i}.x(3, i));
    end

    % Check if the new z_min (refered to another chute) is below the old one. If it is the case, then restore the old one: at the end I want the higher z_min possible
    if agents{i}.z_min < agents{i}.z_min_old
      agents{i}.z_min = agents{i}.z_min_old;
    end

    % Voronoi in xy plane
    if dist <= agents{i}.Rc + coverage*(unc_j + my_unct) && dist_z_norm <= agents{i}.Rcv + unc_z + dir_z*(agents{i}.vmaxzdt - agents{j}.vmaxzdt)
      % If the agents are close enough, then add it on the list of agents seen by the voronoi
      agents{i}.agents_x_voronoi = [agents{i}.agents_x_voronoi agents{i}.x(1:2, j)];

      % Check wether or not the agent have to be moved in order to ensure that during the voronoi tessellation no collisions occur. In particular the agents have to be moved if in one movement (ie given by the maximum distance that can be covered in one time step) they can reach the edge of the voronoi cell. When we move agent j closer to i we have to pay attention in not move it behind i
      if agents{i}.vmaxdt >= dist/2 - agents{i}.delta
        agents{i}.agents_x_voronoi(:, end) = agents{i}.x(1:2, j) + min(2*(agents{i}.delta + epsilon), dist - epsilon)*dir;
      end
      
    end
    
    % In the direction where we don't see any agent we simply reduce the sensing range of an amount equal to the dimension of the agent (eventually increased of its own uncertainty) 
    if agents{i}.vmaxdt >= agents{i}.Rs - agents{i}.delta
      Rs_old = agents{i}.Rs; % save the old sensing range
      agents{i}.Rs = max(epsilon, agents{i}.Rs - agents{i}.delta);
    end

    agents{i}.x(1:2, j) = old_j_pos; % restore the old dimension of agent j
   end
  end

  % In case of only 1 agent, set its z_min Rsv below it
  if n_agents == 1
    agents{i}.z_min = min(agents{i}.x(3,i) - agents{i}.Rsv + coverage*sqrt(agents{i}.P_est{i}(3,3)), agents{i}.x(3,i));
  end

  % Check the number of neighbors and manage the cases
  if size(agents{i}.agents_x_voronoi, 2) == 0             % no other agents -> go with sensing range only
    points = circle(agents{i}.x(1, i), agents{i}.x(2, i), agents{i}.Rs);
    agents{i}.voronoi = polyshape(points(:,1), points(:,2));
  elseif size(agents{i}.agents_x_voronoi, 2) == 1          % only one agent -> take the line in the middle of the agents
    dir = agents{i}.agents_x_voronoi(1:2, 1) - agents{i}.x(1:2, i); % direction of the line from robot to neighbor
    dir = dir/norm(dir);                                   % normalization of the line
    norm_dir = [-dir(2); dir(1)];                          % normal to dir (i.e. line in the middle of the agents)
    M =  mean([agents{i}.x(1:2, i), agents{i}.agents_x_voronoi(1:2, end)], 2); % middle point

    % Check if the new sensing range is large enough to intersect the line in the middle of the agents
    if agents{i}.Rs < norm(M - agents{i}.x(1:2, i))
      points = circle(agents{i}.x(1, i), agents{i}.x(2, i), agents{i}.Rs); % points of the circle of interest
      agents{i}.voronoi = polyshape(points(:,1), points(:,2));
    else  
      % distance between the middle point and the intersection points
      dist_points = sqrt(agents{i}.Rs^2 - norm(M - agents{i}.x(1:2, i))^2); 
      A = M + norm_dir*dist_points;      % circle-middle line intersection sx
      B = M - norm_dir*dist_points;      % circle-middle line intersection dx
      
      points = circle_sector(agents{i}.x(1, i), agents{i}.x(2, i), A, B); % points of the circular sector of interest
      agents{i}.voronoi = polyshape(points(:,1), points(:,2)); 
    end
  else   % at least 2 agents  -> use voronoi packet
    % Add to the first row of agents{i}.agents_x_voronoi the position of the agent itself
    agents{i}.agents_x_voronoi = [agents{i}.x(1:2, i) agents{i}.agents_x_voronoi];
    % agents{i}.agents_x = [agents{i}.x(1:3) agents{i}.agents_x];
    agents{i}.x_idx = [i agents{i}.x_idx];

    % build the limit of the voronoi cell by enlarging the area where the points are deployed of a quantity proportional to the sensing range
    x_min = min(agents{i}.agents_x_voronoi(1,:)) - 1.1*agents{i}.Rs;
    x_max = max(agents{i}.agents_x_voronoi(1,:)) + 1.1*agents{i}.Rs;
    y_min = min(agents{i}.agents_x_voronoi(2,:)) - 1.1*agents{i}.Rs;
    y_max = max(agents{i}.agents_x_voronoi(2,:)) + 1.1*agents{i}.Rs;

    % Perform the voronoi tessellation
    % vert returns the vertices of the voronoi cell associated with the agents in agents{i}.voronoi. The points are given in a cell whom items are in the order of the input points. The points are not given in order, meaning that convhull have to be used in order to rearrange them
    [~, ~, vert] =  voronoiPolyhedrons(agents{i}.agents_x_voronoi, [x_min y_min], [x_max y_max]);
    k = convhull(vert{1}(1,:), vert{1}(2,:));
    poly_voronoi = polyshape(vert{1}(1,k), vert{1}(2,k)); % voronoi cell of i-th agent
    
    % create the polyshape of the sensing circle
    points = circle(agents{i}.x(1, i), agents{i}.x(2, i), agents{i}.Rs);
    poly_circle = polyshape(points(:,1),points(:,2));
  
    % find the intersection between the two polyshapes
    agents{i}.voronoi = intersect(poly_circle, poly_voronoi);
  end
  
  % find th centroid of the cell of the agent itself 
  [agents{i}.centroid_geometric(1), agents{i}.centroid_geometric(2)] = centroid(agents{i}.voronoi);

  % Rewrite the old physical parameters
  if exist('Rs_old')
    agents{i}.Rs = Rs_old;
  end
  if exist('agents_delta')
    delta_final(i) = agents{i}.delta;
    agents{i}.delta = agents_delta;
  end
end

end