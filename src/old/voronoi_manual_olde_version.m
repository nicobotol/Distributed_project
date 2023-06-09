clc
clear 
close all
j_fig = 0;

% INITIAL CONDITION
% the initial distance between the agents has to be larger than the sum of the two deltas

range = 20; % range where the agents are deployed
Rc = 10; % communication range of the robot
Rs = Rc/2; % sensing range of the robot (i.e. where the robot can move at maximum to avoi collisions)
n_agents = 4;  % number of agents
dt = 0.1;
vmax = 30;
agents = cell(n_agents,1);
x = 2*[-1 1 1 -1.0000001];
y = 2*[1 1 -1 -1];
for i = 1:n_agents
%   x = (rand() - 0.5)*range;
%   y = (rand() - 0.5)*range;
  agents{i}.x = [x(i), y(i)]';      % positions of the agents 
  global_positions(i,:) = [x(i),y(i)];  %position of the agents for centralized plot
  agents{i}.neighbors = [];   % neighbors of the agents
  agents{i}.neighbors_pos = []; % positions of the neighbors
  agents{i}.len_n = 0;        % number of neighbors
  agents{i}.vmaxdt = vmax*dt;   % maximum velocity of the agents
  agents{i}.delta = 0.5;     % encumberce of the agent
end

% Check if the agents are within the communication range
for i = 1:n_agents
  for j = 1:n_agents
    if i ~= j
      dist = norm(agents{i}.x - agents{j}.x);
      if dist <= Rc
        agents{i}.neighbors = [agents{i}.neighbors, j];
        agents{i}.len_n = agents{i}.len_n + 1;          % number of agents in Rc
        
        agents{i}.neighbors_pos(end + 1, :) = agents{j}.x;     % positions of the neighbors
        if dist/2 < agents{i}.vmaxdt + agents{i}.delta 
          agents{i}.neighbors_pos(end , :) = agents{j}.x + 2*agents{i}.delta*(agents{i}.x - agents{j}.x)/dist;
        end
      end
    end
  end
end

tic
% Build the dstributed voronoi cell for each agent
for i = 1:n_agents
  Rs_copy = Rs; % reduce the sensing range to take the encumbrance into account
  if agents{i}.vmaxdt + agents{i}.delta > Rs_copy
    Rs_copy = Rs_copy - agents{i}.delta;
  end
  % Initialization of the variables
  P = [];
  vx = [];
  vy = [];
  V = [];
  C = [];
  v = [];
  ia = [];
  inf_points = [];
  robots_pos = [];

  % Check the number of neighbors and manage the cases
  if agents{i}.len_n == 0     % no other agents -> go with sensing range only
    points = circle(agents{i}.x(1), agents{i}.x(2), Rs_copy); % points of the circle of interest
    agents{i}.voronoi = polyshape(points(:,1),points(:,2));
  elseif agents{i}.len_n == 1 % only one agent -> take the line in the middle of the agents
    dir = - agents{i}.x + agents{i}.neighbors_pos'; % direction of the line from robot to neighbor
    dir = dir/norm(dir);                % normalization of the line
    norm_dir = [-dir(2); dir(1)];       % normal to dir (i.e. line in the middle of the agents)
    M =  mean([agents{i}.x'; agents{i}.neighbors_pos], 1)'; % middle point
    if Rs_copy^2 < norm(M - agents{i}.x)^2
      points = circle(agents{i}.x(1), agents{i}.x(2), Rs_copy); % points of the circle of interest
      agents{i}.voronoi = polyshape(points(:,1),points(:,2));
    else
      dist_points = sqrt(Rs_copy^2 - norm(M - agents{i}.x)^2); % distance between the middle point and the intersection points
      A = M + norm_dir*dist_points;      % circle-middle line intersection sx
      B = M - norm_dir*dist_points;      % circle-middle line intersection dx
      
      points = circle_sector(agents{i}.x(1), agents{i}.x(2), A, B); % points of the circular sector of interest
      agents{i}.voronoi = polyshape(points(:,1),points(:,2)); 
    end
  else                        % at least 2 agents  -> use voronoi packet
    % Save the positions of the agents and their neighbors in P (NOTE: the first row is the position of the agent itself)
    P(1,:) = agents{i}.x;
    for j = 1:agents{i}.len_n
      P(j+1,:) = agents{i}.neighbors_pos(j,:);  
    end
    % Compute the voronoi tesselation
    % NOTE:
    % - voronoi gives a set of points (also the "infinite" ones) but not the associations to the agents
    % - voronoin gives the associations to the agents but not the infinite points
    [vx,vy] = voronoi(P(:,1), P(:,2));
    [V,C] = voronoin(P);

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
      inf_points(j, :) = inf_points(j,:) + (inf_points(j,:) - p_linked)/norm(inf_points(j,:) - p_linked)*Rs*100;
      V = [V; inf_points(j,:)]; % add the infinite point to V
    end

    % Associate the new points to the agents
    row_start = length(V(:,1)) - length(inf_points(:, 1)) + 1; % row where the infinite points start in V

    % Loop over the new points
    % for each point we check which is the closest agent
    % if the closest agent is the agent itself, we add the point to C{1}
    % NOTE: C is compute by each agent and the first row is always the agent itself so we
    % have to care only about the first row
    for j = row_start:length(V(:,1))
      V_dist = sum(abs(V(j,:)-P).^2,2).^0.5; % vector of distances between the considered point and the agents
      % V_dist is a vector of n elements where n is the number of agents
      V_index = find(V_dist == min(V_dist)); % check the closest agent
      % if the output is a vector of length 2, it means that the point is equidistant from 2 agents
      if length(V_index)== 2
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
    points = circle(agents{i}.x(1), agents{i}.x(2), Rs_copy);
    poly_circle = polyshape(points(:,1),points(:,2));
    % find the intersection between the two polyshapes
    agents{i}.voronoi = intersect(poly_circle, poly_voronoi);
  end
  
  % find th centroid of the cell of the agent itself 
  [agents{i}.centroid(1), agents{i}.centroid(2)] = centroid(agents{i}.voronoi);
end
toc

% Plot the figure
j_fig = j_fig+1;
figure(j_fig); clf;
hold on
axis equal
for i=1:n_agents
  volume_points = circle(agents{i}.x(1), agents{i}.x(2), agents{i}.delta);
  hold on
  plot(agents{i}.voronoi);
  plot(agents{i}.x(1), agents{i}.x(2), 'xr', 'MarkerSize', 20);
  plot(agents{i}.centroid(1), agents{i}.centroid(2), 'ob', 'MarkerSize', 10);
  plot(volume_points(:,1), volume_points(:,2), '--r', 'LineWidth',2);
  % text(agents{i}.x(1), agents{i}.x(2), num2str(i), 'FontSize', 10);
end
voronoi(global_positions(:,1), global_positions(:,2))


