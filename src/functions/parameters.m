function par = parameters(variable_param)

  %% Simulation parameters
  par.dt =    1;                               % time steep [s]
  par.sim_t = 1700;                            % simulation time [s]
  par.target = [0 0 0]';                       % target point [x y z] [m m m]
  par.Sigma = 10e0*eye(2);                     % std of the distribution used for voronoi centroid navigation

  %% Parachute parameters
  par.n_agents = 5;                            % number of agents
  par.Delta = 5;                               % agent dimension radius [m]
  par.position_range = par.Delta*50;               % range where the agents are deployed
  par.Rc = 50;                                 % communication range of the robot
  par.Rs = par.Rc/2;                               % sensing range of the robot (i.e. where the robot can move at maximum to avoid collisions)
  par.Rcv = 50;                                % communication range of the robot in the vertical directions
  par.Rsv = par.Rcv/2;                             % sensing range of the robot in the vertical directions
  par.z_th = 10;                               % height of the parachute
  if par.z_th > par.Rsv
    error('z_th must be smaller than Rsv')
  end
  par.Beta = 0.1;                              % ratio between viscous coefficient and the chute mass
  par.V_plane = 55;                            % plane speed (initial chutes' speed) [m/s]
  par.V_max = 13;                              % [m/s] maximum forward speed achievable by the chute control (it is not the plane speed)
  par.omega_max = 5;                           % [rad/s] max angular speed
  par.v_lim = 4.87;                            % free falling speed [m/s]
  par.V_z = -par.v_lim;                            % free falling speed [m/s]
  par.vz_min = 1.22;                           % minimum speed [m/s]

  %% Simulation settings
  par.T = par.sim_t/par.dt;                    % number of iterations [-]
  par.t_vect = par.dt:par.dt:par.sim_t;        % [s]

  par.measure_len = 3;                         % number of measurements
  par.R_GPS_scale = (5/3)^2;                   % GPS measurements error, taking into account 2m of error with a covering factor of 3
  par.R_compass_scale = 0.018;                 % compass measurements noise
  par.R_relative = (1/3)^2;                    % relative measurements noise: UWB+camera error, taking into account 1m of
                                              % error with a covering factor of 3
  % wind noise:
  % - calm: 0 to 12 km/h (0 to 3.3 m/s)
  % - light air: 13 to 30 km/h (3.4 to 8.3 m/s)
  % - windy: 31 to 40 km/h (8.4 to 11.1 m/s)
  par.L_scale = (3*par.dt/3)^2;            
  % compass wind disturbance (5 degrees/s = 0.087 rad/s is the maximum speed at which the wind can made the chute rotate)
  par.L_compass_scale = (0.087*par.dt/3)^2;  
  par.n = par.n_agents;                            % number of parachudes
  par.m = 1000;                                % protocol to exchange to reach the consensus
  par.P_est_init = 1e3;                        % random initial position covariance value
  par.IK = 1;                                  % 1 enables the use of the inverse kinamtic in the computtation of the
                                          % position of the global centroid, 0 moves the local centroid assigning the same input of the global one 

  %% Dynamics parameters
  par.nu_mag = 1;                              % magnitude of the noise on the not controllable input

  par.coverage = 3;                            % coverage factor for the increasing of the uncertainty 
  par.epsilon = 1e-3;                          % small value for the voronoi cell correction
  par.coverage_dropout = 3;                    % coverage factor for the exclusion of an agent from the one update with the model  
  par.prob_conn_vec = [0.7 0.8 0.9 1];
  par.prob_connection = par.prob_conn_vec(variable_param.prob_conn); % probability that 2 agents can communicate during the consensus
  par.prob_conn_len = size(par.prob_conn_vec, 2);
  par.prob_rel_measurement_vec = [0.7 0.8 0.9 1];
  par.prob_rel_measurement = par.prob_rel_measurement_vec(variable_param.prob_rel_measurement); % probability of making the realtive measurement between 2 agents
  par.prob_rel_measurement_len = size(par.prob_rel_measurement_vec, 2);
  par.prob_GPS_vec = [0.7 0.8 0.9 1];
  par.prob_GPS = par.prob_GPS_vec(variable_param.prob_GPS);   % probability of making the GPS measurement
  par.prob_GPS_len = size(par.prob_GPS_vec, 2);

  %% Model choice
  par.mdl = 6;                                 % [2, 4, 5, 6] choice of the model SCANF?
  switch par.mdl

    case 5 % unicylce model on the 2D plane and control in z
      
      par.x0 = [100 100 500]';                 % points around which the initial centroid is deployed [x y z]'
      
      par.states_len = 4;                      % numer of states
      par.inputs_len = 3;                      % number of inputs
      par.nc_inputs_len = 5;                   % number of not controllable inputs 
      par.centroid_states_len = 3;             % number of states of the centroid dynamic 
      
      % Matrix for the linear centroid 
      par.A = eye(3);                          % state matrix
      par.B = par.dt*eye(3);                       % input matrix
      
      par.nu_unc = zeros(4, 1);                % uncertainty on the not controllable inputs
      
      par.V_min = 2;                           % [m/s] minimum forward speed 
      par.K_v = 1;                             % speed proportional gain for the low level control
      par.K_omega = par.omega_max/(2*pi);          % angular speed proportional gain for the low level control, saturated

      par.Q_scale_V = ((5/100*par.V_max)/3)^2;     % input measurements noise, taking into account 5% of the maximum speed as the 
                                          % desired standard deviation with a covering factor of 3
      par.Q_scale_omega = ((5/100*par.omega_max)/3)^2;
      par.Q_scale_vz = ((5/100*par.v_lim)/3)^2;

    case 6 
      % linear model with displacement control on x, y, and z
      
      par.x0 = [30 30 5]';                   % points around which the initial centroid is deployed [x y z]'
      
      par.states_len = length(par.x0);             % numer of states
      par.inputs_len = 3;                      % number of inputs
      par.nc_inputs_len = 4;                   % number of not controllable inputs 
      par.centroid_states_len = 3;             % number of states of the centroid dynamic 
      
      par.A = eye(par.states_len);                 % state matrix
      par.B = par.dt*eye(par.inputs_len);              % input matrix
      par.G = eye(3,3);                        % noise matrix
      par.G(:,4) = [0; 0 ;par.dt];                 % add the input to the disturbances
      par.nu_unc = zeros(par.nc_inputs_len, 1);    % uncertainty on the not controllable inputs
      par.Q_scale_vx = ((5/100*par.V_max)/3)^2;    % input measurements noise, taking into account 5% of the maximum speed as the
                                          % desired standard deviation with a covering factor of 3
      par.Q_scale_vy = ((5/100*par.V_max)/3)^2;
      par.Q_scale_vz = ((5/100*par.v_lim)/3)^2;

      par.kp = 1/par.dt;                           % proportional gain for the velocity control

  end

  par.ground_th = 1/10*par.x0(3);                  % distance from the ground to decelerate the agent

  %% Control settings LQR
  par.S = 1*eye(2);                            % weight for states
  par.R = 10*eye(2);                           % weight for inputs
  par.Sf = 5*eye(2);                           % weight for final state

  %% Plots settings
  par.marker_size = 10;
  par.line_width = 2;
  % colors
  par.colors_vect = [[0 0.4470 0.7410]; [0.8500 0.3250 0.0980]; ...
                [0.9290 0.6940 0.1250]; [0.4940 0.1840 0.5560]; ...
                [0.4660 0.6740 0.1880]; [0.3010 0.7450 0.9330]; ...
                [0.6350 0.0780 0.1840]];
  par.enable_video = 0;                        % 1 for enabling, 0 otherwise

  rng(6);                  % random number generator seed

end