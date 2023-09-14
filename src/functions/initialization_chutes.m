function [agents, ground_check, true_centroid_store, w_store] = initialization_chutes(par)
  % This function initializes the agents and the parameters of the simulation

  position_range = par.position_range;
  x0 = par.x0;
  mdl = par.mdl;
  if mdl == 1
    kp = par.kp;
    G = par.G;
    Q_scale_vx = par.Q_scale_vx;
    Q_scale_vy = par.Q_scale_vy;
    Q_scale_vz = par.Q_scale_vz;
  elseif mdl == 2
    V_min = par.V_min;
    K_v = par.K_v;
    K_omega = par.K_omega;
    Q_scale_V = par.Q_scale_V;
    Q_scale_omega = par.Q_scale_omega;
    Q_scale_vz = par.Q_scale_vz;
  end
  dt = par.dt;
  V_max = par.V_max;
  states_len = par.states_len;
  z_th = par.z_th;
  Delta = par.Delta;
  v_lim = par.v_lim;
  vz_min = par.vz_min;
  Rc = par.Rc;
  Rcv = par.Rcv;
  Rsv = par.Rsv;
  Rs = par.Rs;
  R_GPS_scale = par.R_GPS_scale;
  R_compass_scale = par.R_compass_scale;
  L_scale = par.L_scale;
  L_compass_scale = par.L_compass_scale;
  coverage = par.coverage;
  P_est_init = par.P_est_init;
  T = par.T;
  n_agents = par.n_agents;
  measure_len = par.measure_len;
  nc_inputs_len = par.nc_inputs_len;
  inputs_len = par.inputs_len;
  R_relative = par.R_relative;

  agents = cell(n_agents,1);
  w_store = [];
  for i = 1:n_agents
    %% States and inputs parameters
    % Coordinate systems parameters
    if i < 4
      x = (rand() - 0.5)*position_range + x0(1);
      y = (rand() - 0.5)*position_range + x0(2);
      z = (rand() - 0.5)*position_range + x0(3);
    else 
      x = (rand() - 0.5)*position_range - 2*x0(1);
      y = (rand() - 0.5)*position_range - x0(2);
      z = (rand() - 0.5)*position_range + x0(3);
    end

    % add the angle as 4th state if the model is the unicycle
    if mdl == 2
      theta = (rand() - 0.5)*pi/2;
    end

    agents{i}.x_real = [x, y, z]';                          % real positions of the agents 

    if mdl == 2                                           
      agents{i}.x_real(end + 1) =  theta';   
    end

    agents{i}.x_store = agents{i}.x_real;                    % store the position of the agents
    agents{i}.x_real_store = agents{i}.x_real;               % store the position of the agents
    agents{i}.u_store = zeros(inputs_len, 1);                % store the position of the agents
    agents{i}.x = i*ones(states_len, n_agents);              % estimated positions of the agents
    agents{i}.x(:, i) = agents{i}.x_real;                    % estimated positions of the agents
    agents{i}.x_i_previous = agents{i}.x_real;               % estimated state of an agent (not affected by the WLS on it)
    agents{i}.u = zeros(inputs_len, 1);                      % inputs of the agents  
    agents{i}.u_bar = zeros(inputs_len, 1);                  % inputs of the agents  
    agents{i}.u_bar_store = zeros(inputs_len, 1);            % inputs of the agents  
    agents{i}.u_visit = zeros(inputs_len, n_agents);         % last inputs of the neighbors agents
    agents{i}.P_est = cell(n_agents, 1);                     % state covariance matrix
    agents{i}.P_est{i} = 3*1e3*0.001*eye(states_len, states_len);  % state covariance matrix of the agent on itself 
    if mdl == 2 % add the compass uncertatinty
      agents{i}.P_est{i}(states_len, states_len) = R_compass_scale;
    end
    for j=1:n_agents 
      if j~=i
        % state covariance matrix of the other agents (quite high value)
        agents{i}.P_est{j} = P_est_init*eye(states_len, states_len); 
      end
    end
    agents{i}.P_print = cell(T, 1);
    agents{i}.P_est_previous = agents{i}.P_est{i};           % covariance before the WLS 

    %% Parameters for control
    if mdl == 1
      agents{i}.kp = kp;                                     % proportional gain for low level control
    end
    agents{i}.V_max = V_max;                                 % maximum velocity of the agents in x and y direction
    agents{i}.vmaxdt = agents{i}.V_max*dt;                   % maximum velocity of the agents
    agents{i}.sim_x = agents{i}.x_real; 

    agents{i}.centroid = [0,0]';                             % centroid of the voronoi area weighted by the pdf function
    agents{i}.global_centroid = ones(3, 1);
    agents{i}.centroid_geometric = [0,0]';                   % pure geometrical centroid of the voronoi cell
    
    if mdl == 2
      agents{i}.motion_predict = [];                         % motion predeicted area for the NL case
    end
    
    %% Physical parameters
    agents{i}.t_falling = 0;              % time at which the chute opens 
    agents{i}.z_th = z_th;                                   % minimum vertical distance to avoid collisions
    agents{i}.z_min_old = 0;                                 % previous time-step z_min                   
    agents{i}.vmaxzdt = v_lim*dt;                            % max displacement of the agent done in 1 time step in the vertical direction
    agents{i}.delta = Delta;                                 % encumberce of the agent
    agents{i}.vz = 0;                                        % z speed of parachutes
    agents{i}.vz_old = 0;                                    % previous time-step vz
    agents{i}.vz_max = -v_lim;                               % maximum falling velocity
    agents{i}.vz_min = -vz_min;                              % minimum falling velocity
    agents{i}.terminal_speed = 0;                            % 0 if it's still accelerating, 1 if it has reached the terminal speed
    agents{i}.Rs = Rs;                                       % sensing range of the agent
    agents{i}.Rc = Rc;                                       % communication range of the agent
    agents{i}.Rcv = Rcv;                                     % communication range of the agent in the vertical direction
    % sensing range of the agent in the vertical direction (it mst be higherthan then the highest parachute)
    agents{i}.Rsv = Rsv;                   
    
    %% Parameters for voronoi
    agents{i}.neighbors = [];                                % neighbors of the agents
    agents{i}.len_n = 0;                                     % number of neighbors
    % estimate position of the other agents insside the comm. range used for the Voronoi tessellation
    agents{i}.agents_x_voronoi = [];       
    agents{i}.x_idx = [];
    agents{i}.visited_chutes = [];
    
    %% Measuerement instrument parameters
    agents{i}.R_relative = R_relative*eye(3);                % covariance of the relative position measurement
    agents{i}.R_GPS = R_GPS_scale*eye(states_len);           % covariance of the GPS measurement
    if mdl == 2                                              % add the compass uncertatinty
      agents{i}.R_GPS(states_len, states_len) = R_compass_scale;
    end
    agents{i}.Q = eye(inputs_len);                           % covariance of the input measurement
    if mdl == 1                                              % linear model
      agents{i}.Q(1,1) = Q_scale_vx;
      agents{i}.Q(2,2) = Q_scale_vy;
      agents{i}.Q(3,3) = Q_scale_vz;
    else                                                     % unicycle model
      agents{i}.Q(1,1) = Q_scale_V;
      agents{i}.Q(2,2) = Q_scale_omega;
      agents{i}.Q(3,3) = Q_scale_vz;
    end
    agents{i}.L = L_scale*eye(nc_inputs_len);                % covariance of the GPS measurement
    agents{i}.L(nc_inputs_len, nc_inputs_len) = L_compass_scale;
    agents{i}.H_GPS = eye(states_len);                       % measurement matrix for GPS
    agents{i}.nu = zeros(nc_inputs_len, 1);
    agents{i}.H = eye(measure_len*n_agents, measure_len*n_agents); % measurement matrix for the relative position
  end

  % Check if each agent does not touch the others in the initial position
  for i = 1:n_agents
    for j = i+1:n_agents
      dir = agents{i}.x_real(1:2) - agents{j}.x_real(1:2);   % direction between agents
      dist = norm(dir);                                      % distance between agents in 2D plane
      sign_z = agents{i}.x_real(3) - agents{j}.x_real(3);
      dist_z = abs(sign_z);                                  % distance between 2 agents in the vertical direction
      if dist_z <= agents{i}.Rcv && dist <= (agents{i}.delta + agents{j}.delta + agents{i}.vmaxdt + agents{j}.vmaxdt + 2*coverage*sqrt(R_GPS_scale))

        agents{j}.x_real(1:2) = agents{j}.x_real(1:2) + 2*(1 + rand())*agents{j}.delta*(-dir/dist);
        agents{j}.x_real(3) = agents{j}.x_real(3) + 2*(1 + rand())*agents{j}.Rcv; % move the agent upword


        agents{j}.x_real_store(:) = agents{j}.x_real; 
        agents{j}.x(:, j) = agents{j}.x_real; 
        agents{j}.x_store(:) = agents{j}.x_real; 
        agents{j}.x_i_previous = agents{j}.x_real;
        
      end 
    end
    if mdl == 2
      agents{i}.x(4, i) = 0;
      agents{i}.x_real(4) = 0;
    end
    agents{i}.x_i_previous = agents{i}.x_real;
  end

  %% Simulation parameters
  ground_check = zeros(n_agents, 1);    % each element is 1 if the corresponding chute has touch the ground, 0 otherwise
  true_centroid_store = zeros(3, 1);    % global centroid position
  w_store = 0;

end