%% This functon computes the Voronoi cell for each agent
function [agents, delta_final] = voronoi_chutes(agents)

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
        if mdl == 6
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
            agents{i}.agents_x_voronoi(:, end) = agents{i}.x(1:2, j) + 2*(agents{i}.delta - epsilon)*dir;
          end
          
        end
        agents{j}.x(1:2, j) = old_j_pos; % restore the old dimension of agent j
      end
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
      agents{i}.x_idx = [i agents{i}.x_idx] ;
  
      % Compute the voronoi tesselation
      % NOTE:
      % - voronoi gives a set of points (also the "infinite" ones) but not the associations to the agents
      % - voronoin gives the associations to the agents but not the infinite points
      [vx,vy] = voronoi(agents{i}.agents_x_voronoi(1,:)', agents{i}.agents_x_voronoi(2,:)');
      [V,C] = voronoin(agents{i}.agents_x_voronoi');
  
      % remove infinite values in V (if there are any they are in the first row)
      if isinf(V(1,1)) || isinf(V(1,2)) 
        V(1,:) = []; % remove the first row of V
        C{1}(find(C{1} == 1)) = []; % remove the number 1 from C{1}
        C{1} = C{1} - 1;      % decrement all the other numbers of C{1} since we removed the first row of V
      end
  
      % create a matrix of the points given by voronoi
      v = zeros(length(vx(1,:))*2,2);
      v = [vx(1,:)', vy(1,:)'; vx(2,:)', vy(2,:)']; 
      % remove the duplicate points
      v = unique(v, 'rows'); 
      
      % compare V and v to add the infinite points to V
      [~, ia] = setdiff(round(v, 6), round(V,6), 'rows'); % a rounding is needed -> there are some small numerical issues
      inf_points = v(ia,:); % ia are the indices of the infinite points in v (points that are in v but not in V)
  
      % NOTE: the infinite points need to be elongated in order to perform the intersection with the sensing range
      % so we need to reconstruct the direction of the line checking the associated points in vx and vy
      % associate the infinite points to vx and vy and elongate them
      
      % Loop over the infinite points 
      for j = 1:length(inf_points(:,1))
        [r_inf,c_inf] = find(vx == inf_points(j,1)); % find the row and column of the infinite point in vx
        % if r_inf is not unique, check the y
        if length(r_inf) > 1
          [r_inf,c_inf] = find(vy == inf_points(j,2));
        end
        
        if r_inf == 1
          p_linked = [vx(2,c_inf), vy(2,c_inf)]; % point linked to the infinite point
        else
          p_linked = [vx(1,c_inf), vy(1,c_inf)];
        end
        % elongate the infinite point towards infinity
        inf_points(j, :) = inf_points(j,:) + (inf_points(j,:) - p_linked)/norm(inf_points(j,:) - p_linked)*agents{i}.Rs*100;
        V = [V; inf_points(j,:)]; % add the infinite point to V
      end
  
      % Associate the new points to the agents
      row_start = length(V(:,1)) - length(inf_points(:, 1)) + 1; % row where the infinite points start in V
      % Save the positions of the agents and their neighbors in robots_pos 
      % (NOTE: the first row is the position of the agent itself)
  
  
      % Loop over the new points
      % for each point we check which is the closest agent
      % if the closest agent is the agent itself, we add the point to C{1}
      % NOTE: C is compute by each agent and the first row is always the agent itself so we
      % have to care only about the first row
      for j = row_start:length(V(:,1))
        V_dist = sum(abs(V(j,:)-agents{i}.x(1:2,agents{i}.x_idx)').^2,2).^0.5; % vector of distances between the considered point and the agents
        % V_dist is a vector of n elements where n is the number of agents
        V_index = find(V_dist == min(V_dist)); % check the closest agent
        % if the output is a vector of length 2, it means that the point is equidistant from 2 agents
        if length(V_index) == 2
          % if the point is close to the agent itself, add it to C{1}
          if length(find(V_index == 1)) == 1
            C{1} = [C{1}, j];
          end
        else % if the output is a vector of length 1, it means that the point is close to only one agent (numerical issues)
          V_dist(V_index) = max(V_dist); % set the first minimum to the maximum in order to not take it twice
          if length(find(V_index == 1)) == 1 % if the closest agent is the agent itself, add the point to C{1}
            C{1} = [C{1}, j];
            continue; % if the closest agent is the agent itself, go to the next point (no need to check the second minimum)
          end
          V_index = find(V_dist == min(V_dist)); % find the second minimum if the first one is not the agent itself
          if length(find(V_index == 1)) == 1
            C{1} = [C{1}, j];
          end
        end
      end
  
      % create the polyshape of the cell
      k = convhull(V(C{1},:)); % take the points in a order such that they form a convex polygon
      % take the point of V associated to the agent itself in the order given by k
      poly_voronoi = polyshape(V(C{1}(k), 1), V(C{1}(k), 2));
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