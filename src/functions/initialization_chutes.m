function [agents, ground_check, true_centroid_store] = initialization_chutes()


  % Load the parameters
  parameters;
  % x = [0 0];
  % y = [0 1];
  % z = [0 0 3];
  agents = cell(n_agents,1);
  for i = 1:n_agents
    %% Coordinate systems parameters
    if i < 3
      x = (rand() - 0.5)*position_range + x0(1);
      y = (rand() - 0.5)*position_range + x0(2);
      z = (rand() - 0.5)*position_range + x0(3);
    else 
      x = (rand() - 0.5)*position_range - 2*x0(1);
      y = (rand() - 0.5)*position_range - x0(2);
      z = (rand() - 0.5)*position_range + x0(3);
    end

    if mdl == 4 % model 4
      theta = (rand() - 0.5)*pi/2;
    end

    agents{i}.x_real = [x, y, z]';  % real positions of the agents 

    if mdl == 4 % add the angle as 4th state
      agents{i}.x_real(end + 1) =  theta';   
    end

    agents{i}.x_store = agents{i}.x_real; % store the position of the agents
    agents{i}.x_real_store = agents{i}.x_real; % store the position of the agents
    agents{i}.u_store = zeros(inputs_len, 1); % store the position of the agents
    
    agents{i}.x = i*ones(states_len, n_agents);  % estimated positions of the agents
    agents{i}.x(:, i) = agents{i}.x_real;  % estimated positions of the agents
    agents{i}.x_i_previous = agents{i}.x_real; % estimated state of an agent (not affected by the WLS on it)
    agents{i}.u = zeros(inputs_len, 1);         % inputs of the agents  
    agents{i}.u_visit = zeros(inputs_len, n_agents);         % last inputs of the neighbors agents
    agents{i}.kp = kp;                         % proportional gain for low level control
    % agents{i}.x_real = [x(i), y(i), z(i)]'; % real positions of the agents 
    agents{i}.sim_x = agents{i}.x_real; 
    agents{i}.P_est = cell(n_agents, 1); % state covariance matrix
    agents{i}.P_est{i} = R_GPS_scale*eye(states_len, states_len); % state covariance matrix of the agent on itself 
    if mdl == 4 % add the compass uncertatinty
      agents{i}.P_est{i}(states_len, states_len) = R_compass_scale;
    end

    for j=1:n_agents 
      if j~=i
        agents{i}.P_est{j} = P_est_init*eye(states_len, states_len); % state covariance matrix of the other agents (quite high value)
      end
    end
    agents{i}.P_print = cell(T, 1);
    agents{i}.P_est_previous = agents{i}.P_est{i}; % covariance before the WLS 
    agents{i}.P_DKF = 10*ones(n_agents*states_len);
    agents{i}.centroid = [0,0]';         % centroid of the voronoi area weighted by the pdf function
    agents{i}.global_centroid = ones(3, 1);
    agents{i}.centroid_geometric = [0,0]'; % pure geometrical centroid of the voronoi cell
    
    %% Physical parameters
    agents{i}.z_th = z_th;        % minimum vertical distance to avoid collisions
    agents{i}.vmaxdt = vmax*dt;   % maximum velocity of the agents
    agents{i}.delta = Delta;        % encumberce of the agent
    agents{i}.v_max = -v_lim;      % maximum falling velocity
    agents{i}.v_min = -v_min;      % minimum falling velocity
    %% Parameters for voronoi
    agents{i}.neighbors = [];   % neighbors of the agents
    agents{i}.len_n = 0;        % number of neighbors
    % agents{i}.agents_x = [];      % estimate of the positosion of the others agents
    % agents{i}.agents_P_est = cell(n_agents-1,1); % state covariance matrix of other agents
    agents{i}.agents_x_voronoi = []; % estimate position of the other agents insside the comm. range used for the Voronoi tessellation
    agents{i}.x_idx = [];
    agents{i}.visited_chutes = [];
    
    %% Measuerement instrument parameters
    agents{i}.Rs = Rs; % sensing range of the agent
    agents{i}.Rc = Rc; % communication range of the agent
    agents{i}.Rcv = Rcv; % communication range of the agent in the vertical direction
    agents{i}.Rsv = Rsv; % sensing range of the agent in the vertical direction (it mst be higher than then the highest parachute)
    agents{i}.R_relative = R_relative*eye(3); % covariance of the relative position measurement
    agents{i}.R_GPS = R_GPS_scale*eye(states_len); % covariance of the GPS measurement
    if mdl == 4 % add the compass uncertatinty
      agents{i}.R_GPS(states_len, states_len) = R_compass_scale;
    end
    agents{i}.Q = Q_scale*eye(inputs_len); % covariance of the input measurement
    agents{i}.L = L_scale*eye(nc_inputs_len);        % covariance of the GPS measurement
    agents{i}.H_GPS = eye(states_len); % measurement matrix for GPS
    agents{i}.nu = zeros(nc_inputs_len, 1);
    agents{i}.nu_unc = zeros(nc_inputs_len, 1);    % uncertainty of the inputs
    agents{i}.H = eye(measure_len*n_agents, measure_len*n_agents); % measurement matrix for the relative position in the DKF
%     agents{i}.H = eye(measure_len, measure_len);
  end

  % Check if each agent does not touch the others in the initial position
  for i = 1:n_agents
    for j = i+1:n_agents
      dir = agents{i}.x_real(1:2) - agents{j}.x_real(1:2); % direction between agents
      dist = norm(dir); % distance between agents in 2D plane
      sign_z = agents{i}.x_real(3) - agents{j}.x_real(3);
      dist_z = abs(sign_z); % distance between 2 agents in the vertical direction
      if dist_z <= agents{i}.Rcv && dist <= (agents{i}.delta + agents{j}.delta + agents{i}.vmaxdt + agents{j}.vmaxdt + 2*coverage*sqrt(R_GPS_scale))

        agents{j}.x_real(1:2) = agents{j}.x_real(1:2) + 2*(1 + rand())*agents{j}.delta*(-dir/dist);
        agents{j}.x_real(3) = agents{j}.x_real(3) + 2*(1 + rand())*agents{j}.Rcv; % move the agent upword
        agents{j}.x(1:2, j) = agents{j}.x_real(1:2); 
        agents{j}.x_i_previous(1:2) = agents{j}.x_real(1:2);
      
      end 
    end
  end

  ground_check = zeros(n_agents, 1); % each element is 1 if the corresponding chute has touch the ground, 0 otherwise
  true_centroid_store = zeros(3, 1); % global centroid position

end