function [agents, ground_check, true_centroid_store] = initialization_chutes()

  % Load the parameters
  parameters;
  % x = [0 0];
  % y = [0 1];
  % z = [0 0 3];
  agents = cell(n_agents,1);
  for i = 1:n_agents
    %% Coordinate systems parameters
    if i < 4
    x = (rand() - 0.5)*position_range + x0(1);
    y = (rand() - 0.5)*position_range + x0(2);
    z = (rand() - 0.5)*position_range + x0(3);
    else 
      x = (rand() - 0.5)*position_range - x0(1);
    y = (rand() - 0.5)*position_range - x0(2);
    z = (rand() - 0.5)*position_range + x0(3);
    end
    agents{i}.x_real = [x, y, z]';  % real positions of the agents 
    agents{i}.x_store = agents{i}.x_real; % store the position of the agents
    
    agents{i}.x = zeros(states_len, n_agents);  % estimated positions of the agents
    agents{i}.x(:, i) = agents{i}.x_real;  % estimated positions of the agents
    agents{i}.x_i_previous = agents{i}.x_real; % estimated state of an agent (not affected by the consensus on it)
    agents{i}.u = zeros(inputs_len, 1);         % inputs of the agents  
    agents{i}.kp = kp;                         % proportional gain for low level control
    % agents{i}.x_real = [x(i), y(i), z(i)]'; % real positions of the agents 
    agents{i}.sim_x = agents{i}.x_real; 
    agents{i}.P_est = cell(n_agents, 1); % state covariance matrix
    agents{i}.P_est{i} = 0.5*eye(states_len, states_len);
    for j=1:n_agents 
      if j~=i
        agents{i}.P_est{j} = P_est_init*eye(states_len, states_len);
      end
    end
    agents{i}.P_est_previous = agents{i}.P_est{i}; % covariance before the WLS 
    agents{i}.centroid = [0,0]';         % centroid of the voronoi area weighted by the pdf function
    agents{i}.global_centroid = ones(3, 1);
    agents{i}.centroid_geometric = [0,0]'; % pure geometrical centroid of the voronoi cell
    
    %% Physical parameters
    agents{i}.z_th = z_th;        % minimum vertical distance to avoid collisions
    agents{i}.vmaxdt = vmax*dt;   % maximum velocity of the agents
    agents{i}.delta = Delta;        % encumberce of the agent
    
    %% Parameters for voronoi
    agents{i}.neighbors = [];   % neighbors of the agents
    agents{i}.len_n = 0;        % number of neighbors
    % agents{i}.agents_x = [];      % estimate of the positosion of the others agents
    % agents{i}.agents_P_est = cell(n_agents-1,1); % state covariance matrix of other agents
    agents{i}.agents_x_voronoi = [];
    agents{i}.x_idx = [];
    
    %% Measuerement instrument parameters
    agents{i}.Rs = Rs; % sensing range of the agent
    agents{i}.Rc = Rc; % communication range of the agent
    agents{i}.R_relative = R_relative*eye(3); % covariance of the relative position measurement
    agents{i}.R_GPS = R_GPS_scale*eye(3); % covariance of the GPS measurement
    agents{i}.Q = Q_scale*eye(inputs_len); % covariance of the input measurement
    agents{i}.L = L_scale;         % covariance of the GPS measurement
    agents{i}.H_GPS = eye(states_len); % measurement matrix for GPS
    agents{i}.nu = zeros(4, 1);
  end

  % Check if each robot does not touch the others in the initial position
  for i = 1:n_agents
    for j = i+1:n_agents
      dir = agents{i}.x(1:2, i) - agents{j}.x(1:2, j); % direction between robots
      dist = norm(dir); % distance between robots in 2D plane
      sign_z = agents{i}.x(3, i) - agents{j}.x(3, j);
      dist_z = abs(agents{i}.x(3, i) - agents{j}.x(3, j)); % distance between 2 robots in the vertical direction
      if ((sign_z < 0 && dist_z <= agents{i}.z_th) || (sign_z > 0 && dist_z <= agents{j}.z_th)) && dist <= (agents{i}.delta + agents{j}.delta)
        agents{j}.x(1:2, j) = agents{j}.x(1:2, j) + (1 + rand())*agents{j}.delta*(-dir/dist);
      end 
    end
  end

  ground_check = zeros(n_agents, 1); % each element is 1 if the corresponding chute has touch the ground, 0 otherwise
  true_centroid_store = zeros(3, 1); % global centroid position

end