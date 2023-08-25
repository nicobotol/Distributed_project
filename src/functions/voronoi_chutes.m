%% This functon computes the Voronoi cell for each agent
function agents = voronoi_chutes(agents, t)
t;
parameters;                 % load the parameters
n_agents = length(agents);  % number of agents 

for i = 1:n_agents
  % Initialization of the variables
  vx = [];
  vy = [];
  V = [];
  C = [];
  v = [];
  ia = [];
  inf_points = [];
  inf_points_rescaled = [];
  agents{i}.agents_x_voronoi = [];
  agents{i}.x_idx = [];

  % Increase the encumbrabce of the agent in order to take into account the uncertainty on the knowledge of its own position
  agents_delta = agents{i}.delta;                                             % physical dimension of the agent i
  my_unct = max(sqrt(agents{i}.P_est{i}(1, 1)), sqrt(agents{i}.P_est{i}(2,2)));           % uncertainty on myself
  agents{i}.delta = agents{i}.delta + coverage*my_unct;    % increase the encumbrance of the agent

  % Estimation of the positions of the other robots
  for j = 1:n_agents % loop over all the agents
    if j ~= i
      dist = norm(agents{i}.x(1:2, i) - agents{i}.x(1:2, j)); % distance between robots in 2D plane
      dir = (agents{i}.x(1:2, i) - agents{i}.x(1:2, j))/dist; % direction between i and j

      % old_j_delta = agents{j}.delta; % save the old dimension of agent j
      % unct_j = min(dist - agents{j}.delta - agents{i}.delta, coverage*max(sqrt(agents{i}.P_est{j}(1, 1)), sqrt(agents{i}.P_est{j}(2,2))));
      % agents{j}.delta = agents{j}.delta + unct_j; % inflate the dimsnion of agent j by a quantity equal to the uncertainty that i has on j

      % check how much do we have to make the other robot closer: the minimum between the reciprocal distance and coverage*uncertainty
      unc_j = max(sqrt(agents{i}.P_est{j}(1, 1)), sqrt(agents{i}.P_est{j}(2,2))); % uncertainty in the plane
      old_j_pos = agents{j}.x(1:2, j); % save the old position of agent j
      agents{i}.x(1:2, j) = agents{i}.x(1:2, j) + min(dist - 2*agents{i}.delta, coverage*unc_j)*dir;
      dist = norm(agents{i}.x(1:2, i) - agents{i}.x(1:2, j)); % distance between robots in 2D plane

      dist_z = agents{i}.x(3, i) - agents{i}.x(3, j);
      dist_z_norm = abs(dist_z); % distance between 2 robots in the vertical direction
      unc_z = sqrt(agents{i}.P_est{i}(3,3)) + sqrt(agents{i}.P_est{j}(3,3)); % uncertainty in the z direction

      % Voronoi in z direction
      % Set the voronoi limit in the vertical direction below the agent. Each agent, once sees another one reasonably close to it sets the limit of the voronoi cell in the vertical direction below it. Initially the limit is the sensing range, but then it is moved closer to the agent in order to consider the uncertainty on the position and the velocity of the two 
      if mdl == 6 || mdl == 5
        if dist_z_norm <= agents{i}.Rcv + unc_z && dist_z >= 0 && dist <= agents{i}.Rc
          % agents{i}.z_min = min(agents{i}.x(3, j) + agents{i}.Rsv + (agents{i}.u(3) - agents{i}.u_visit(3, j))*dt + unc_z, agents{i}.x(3, i)); % set that in any case the limit cannot go above the agent in order to avoid negative control inputs
          agents{i}.z_min = min(agents{i}.x(3, j) + agents{i}.Rsv + agents{i}.vmaxzdt + unc_z, agents{i}.x(3, i)); % set that in any case the limit cannot go above the agent in order to avoid negative control inputs
        else
          agents{i}.z_min = agents{i}.x(3, i) - agents{i}.Rsv;
        end
      end
      
      % Voronoi in the xy plane
      if ((dist_z_norm <= agents{i}.Rcv + unc_z) && dist <= agents{i}.Rc) 

        agents{i}.agents_x_voronoi = [agents{i}.agents_x_voronoi agents{i}.x(1:2, j)];
        agents{i}.x_idx = [agents{i}.x_idx j]; %index of the point used for voronoi in the agents{i}.x vector

        if dist/2 <= (agents{i}.vmaxdt + agents{i}.delta) 
          agents{i}.agents_x_voronoi(:, end) = agents{i}.x(1:2, j) + min(dist - 2*agents{i}.delta, 2*(agents{i}.delta - epsilon))*dir;
        end
        
      end
      agents{j}.x(1:2, j) = old_j_pos; % restore the old dimension of agent j
    end
  end

  % In case of only 1 agent, set its z_min Rsv below it
  if n_agents == 1
    agents{i}.z_min = agents{i}.x(3, i) - agents{i}.Rsv;
  end

  % Check if there are problems in the edge limited by the sensing range. If it is the case, then reduce the sensing range
  if agents{i}.vmaxdt + agents{i}.delta > agents{i}.Rs
    Rs_old = agents{i}.Rs; % save the old sensing range
    agents{i}.Rs = max(epsilon, agents{i}.Rs - agents{i}.delta);
  end

  % Check the number of neighbors and manage the cases
  if size(agents{i}.agents_x_voronoi, 2) == 0     % no other agents -> go with sensing range only
    points = circle(agents{i}.x(1, i), agents{i}.x(2, i), agents{i}.Rs);
    agents{i}.voronoi = polyshape(points(:,1), points(:,2));
  elseif size(agents{i}.agents_x_voronoi, 2) == 1 % only one agent -> take the line in the middle of the agents
    dir = agents{i}.agents_x_voronoi(1:2, 1) - agents{i}.x(1:2, i); % direction of the line from robot to neighbor
    dir = dir/norm(dir);                % normalization of the line
    norm_dir = [-dir(2); dir(1)];       % normal to dir (i.e. line in the middle of the agents)
    M =  mean([agents{i}.x(1:2, i), agents{i}.agents_x_voronoi(1:2, end)], 2); % middle point

    % Check if the new sensing range is large enough to intersect the line in the middle of the agents
    if agents{i}.Rs < norm(M - agents{i}.x(1:2, i))
      points = circle(agents{i}.x(1, i), agents{i}.x(2, i), agents{i}.Rs); % points of the circle of interest
      agents{i}.voronoi = polyshape(points(:,1), points(:,2));
    else  
      dist_points = sqrt(agents{i}.Rs^2 - norm(M - agents{i}.x(1:2, i))^2); % distance between the middle point and the intersection points
      A = M + norm_dir*dist_points;      % circle-middle line intersection sx
      B = M - norm_dir*dist_points;      % circle-middle line intersection dx
      
      points = circle_sector(agents{i}.x(1, i), agents{i}.x(2, i), A, B); % points of the circular sector of interest
      agents{i}.voronoi = polyshape(points(:,1), points(:,2)); 
    end
  else                        % at least 2 agents  -> use voronoi packet
    % Add to the first row of agents{i}.agents_x_voronoi the position of the agent itself
    agents{i}.agents_x_voronoi = [agents{i}.x(1:2, i) agents{i}.agents_x_voronoi];
    % agents{i}.agents_x = [agents{i}.x(1:3) agents{i}.agents_x];
    agents{i}.x_idx = [i agents{i}.x_idx];

    % build the limit of the voronoi cell by enlarging the area where the points are deployed of a quantity proportional to the sensing range
    x_min = min(agents{i}.agents_x_voronoi(1,:)) - 1.1*agents{i}.Rs;
    x_max = max(agents{i}.agents_x_voronoi(1,:)) + 1.1*agents{i}.Rs;
    y_min = min(agents{i}.agents_x_voronoi(2,:)) - 1.1*agents{i}.Rs;
    y_max = max(agents{i}.agents_x_voronoi(2,:)) + 1.1*agents{i}.Rs;
    bs = [x_min  x_max x_max x_min; y_min y_min y_max y_max]';

    % This function returns the vertices of the cells in V, while the nodes associated with one cell in C. Since the cells are given in counterclockwise order, later on we have to check which is the one of the poit under study  
    [V,C,~] = VoronoiLimit(agents{i}.agents_x_voronoi(1, :), agents{i}.agents_x_voronoi(2,:), 'bs_ext', bs, 'figure', 'off');

    % check which cell is the one associated with the point under study
    for j=1:size(agents{i}.agents_x_voronoi, 2)
      poly_voronoi = polyshape(V(C{j}, 1), V(C{j}, 2)); % polyshape done basing on the voronoi
      if isinterior(poly_voronoi, agents{i}.x(1:2, i)')
        break
      end
    end

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
    agents{i}.delta = agents_delta;
  end
end

end