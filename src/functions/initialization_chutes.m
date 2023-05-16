function [agents] = initialization_chutes()

  % Load the parameters
  parameters;
  x = [0 0 1];
  y = [0 1 0];
  z = [0 0 0];
  agents = cell(n_agents,1);
  for i = 1:n_agents
    % x = (rand() - 0.5)*position_range;
    % y = (rand() - 0.5)*position_range;
    % z = (rand() - 0.5)*position_range;
    agents{i}.x_real = [x(i), y(i), z(i)]'; % real positions of the agents 
    agents{i}.x = [];                      % estimated positions of the agents 
    agents{i}.P_est = [];                   % state covariance matrix
    agents{i}.eps_cov = [];                 % covariance of the relative position measurement
    % agents{i}.x = [x, y, z]';      % positions of the agents 
    global_positions(i,:) = [x,y,z];  %position of the agents for centralized plot
    agents{i}.neighbors = [];   % neighbors of the agents
    agents{i}.len_n = 0;        % number of neighbors
    agents{i}.centroid_geometric = [0,0]';
    agents{i}.centroid = [0,0]';
    agents{i}.agents_x = [];      % estimate of the positosion of the others agents
    agents{i}.agents_P_est = cell(n_agents-1,1); % state covariance matrix of other agents
    agents{i}.agents_x_voronoi = [];
    agents{i}.agents_x_idx = [];
    agents{i}.vmaxdt = vmax*dt;   % maximum velocity of the agents
    agents{i}.delta = Delta;        % encumberce of the agent
    agents{i}.Rs = Rs;            % sensing range of the agent
    agents{i}.Rc = Rc;            % communication range of the agent
    agents{i}.z_th = z_th;        % minimum vertical distance to avoid collisions
  end

  % Check if each robot does not touch the others in the initial position
  for i = 1:n_agents
    for j = i+1:n_agents
      dir = agents{i}.x(1:2) - agents{j}.x(1:2); % direction between robots
      dist = norm(dir); % distance between robots in 2D plane
      sign_z = agents{i}.x(3) - agents{j}.x(3);
      dist_z = abs(agents{i}.x(3) - agents{j}.x(3)); % distance between 2 robots in the vertical direction
      if ((sign_z < 0 && dist_z <= agents{i}.z_th) || (sign_z > 0 && dist_z <= agents{j}.z_th)) && dist <= (agents{i}.delta + agents{j}.delta)
        agents{j}.x(1:2) = agents{j}.x(1:2) + (1 + rand())*agents{j}.delta*(-dir/dist);
      end 
    end
  end

end