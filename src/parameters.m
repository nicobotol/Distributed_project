%% Constant parameters for the simualtion

%% Simulation parameters
dt = 0.1;          % time steep [s]
sim_t = 20;         % simulation time [s]
target = [0 0 0]';  % target point [x y z] [m m m]
Sigma = 10e0*eye(2);     % std of the distribution used for navigation

%% Parachute parameters
n_agents = 1;       % number of agents
position_range = 50;% range where the agents are deployed
Rc = 20;             % communication range of the robot
Rs = Rc/2;          % sensing range of the robot (i.e. where the robot can move at maximum to avoi collisions)
Rcv = 10;         % communication range of the robot in the vertical directions
Rsv = Rcv/2;      % sensing range of the robot in the vertical directions
z_th = 4.5;           % height of the parachute
if z_th > Rsv
  error('z_th must be smaller than Rsv')
end
Delta = 1;         % agent dimension radius
vmax = 10.0;       % maximum velocity of the agent
kp = 10;           % proportional gain for the velocity control
Beta = 1;          % ratio between viscous coefficient and the chute mass

%% Simulation settings
T = sim_t/dt;             % number of iterations [-]
t_vect = dt:dt:sim_t;     % [s]
Q_scale = 0;              % input measurements noise
Q_bias = 0.5;
measure_len = 3;          % number of measurements
R_GPS_scale = 0.001;
R_GPS_bias = 0;
R_compass_scale = 1e-6;   % compass measurements noise
R_relative = 0;           % relative measurements noise
L_scale = 0.0;            % external disturbance
L_bias = 0.5;
n = n_agents;             % number of parachudes
m = 1000;                 % protocol to exchange to reach the consensus
P_est_init = 1e3;         % random initial position covariance value
IK = 1; % 1 enables the use of the inverse kinamtic in the computtation of the position of the global centroid, 0 moves the local centroid assigning the same input of the global one 
% P_est_threshold = norm(P_est_init*eye(states_len, states_len)); % threshold for the covariance matrix to ignore far agents
%% Dynamics parameters
nu_mag = 1;   % magnitude of the noise on the not controllable input
v_lim = 5;   % fre falling speed [m/s]
V_z = -v_lim; % free falling speed [m/s]
vz_min = 5;    % minimum speed [m/s]
coverage = 3; % coverage factor for the increasing of the uncertainty 
epsilon = 1e-3; % small value for the voronoi cell correction
coverage_dropout = 3; % coverage factor for the exclusion of an agent from the one update with the model  
prob_connection = 0.8; % probability of connection between two agents
prob_communication = 0.8; % probability of communication between two agents

%% Model choice
mdl = 5; % [2, 4, 5, 6] choice of the model
if mdl == 2 
  % linear model with displacement control on x and y
  
  x0 = [30 30 40]';   % points around which the initial centroid is deployed [x y z]'
  
  states_len = length(x0);  % numer of states
  inputs_len = 2;           % number of inputs
  nc_inputs_len = 4;        % number of not controllable inputs
  centroid_states_len = 3;  % number of states of the centroid dynamic 
  
  A = eye(states_len);      % state matrix
  B = [dt 0;
  0 dt;
  0 0 ];                    % input matrix
  G = eye(3,3);             % noise matrix
  G(:,4) = [0; 0 ;dt];      % add the input to the disturbances
  nu_unc = zeros(nc_inputs_len, 1);   % uncertainty on the not controllable inputs

elseif mdl == 5 % unicylce model on the 2D plane and control in z
  
  x0 = [30 30 60]';   % points around which the initial centroid is deployed [x y z]'
  
  states_len = 4;           % numer of states
  inputs_len = 3;           % number of inputs
  nc_inputs_len = 5;        % number of not controllable inputs 
  centroid_states_len = 3;  % number of states of the centroid dynamic 
  
  % Matrix for the linear centroid 
  A = eye(3);               % state matrix
  B = dt*eye(3);            % input matrix
  % G = eye(3,3);           % noise matrix
  % G(:,4) = [0; 0 ;dt];    % add the input to the disturbances
  nu_unc = zeros(4, 1);     % uncertainty on the not controllable inputs
  
  % Linearized matrix for the unicycle model
  syms A_lin(v, theta) B_lin(theta)
  A_lin(v, theta) = [1 0 0 -sin(theta)*v*dt; 
                      0 1 0 cos(theta)*v*dt;
                      0 0 1 0;
                      0 0 0 1];
  B_lin(theta) = [cos(theta)*dt 0 0;
                  sin(theta)*dt 0 0;
                  0 0 dt;
                  0 dt 0];
  V_min = 0.5 ; % [m/s] minimum forward speed 
  V_max = 20; % [m/s] maximum forward speed 
  omega_max = 5; % [rad/s] max angular speed
  K_v = 1;  % speed proportional gain for the low level control
  K_omega = omega_max/(2*pi); % angular speed proportional gain for the low level control, saturated

elseif mdl == 6 
  % linear model with displacement control on x, y, and z
  
  x0 = [30 30 60]';   % points around which the initial centroid is deployed [x y z]'
  
  states_len = length(x0);  % numer of states
  inputs_len = 3;           % number of inputs
  nc_inputs_len = 4;        % number of not controllable inputs 
  centroid_states_len = 3;  % number of states of the centroid dynamic 
  
  A = eye(states_len);                % state matrix
  B = dt*eye(inputs_len);  % input matrix
  G = eye(3,3); % noise matrix
  G(:,4) = [0; 0 ;dt]; % add the input to the disturbances
  nu_unc = zeros(nc_inputs_len, 1);   % uncertainty on the not controllable inputs

elseif mdl == 4
  % NL model with only rotation control, fix forward speed
  
  x0 = [30 30 40 0]';   % points and orientation around which the agents are initially deployed [x y z theta]'
  V = 10;
  
  states_len = length(x0);  % numer of states
  inputs_len = 1;           % number of inputs
  nc_inputs_len = 6;        % number of not controllable inputs
  centroid_states_len = 3;  % number of states of the centroid dynamic 
  
  A = eye(states_len);                % state matrix
  B = [0 0 0 dt]';  % input matrix
  syms G(theta)
  G(theta) = [-sin(theta)*dt 1 0 0 0 0;
  cos(theta)*dt 0 1 0 0 0;
  0 0 0 1 dt 0;
  0 0 0 0 0 1];
  nu_unc = zeros(nc_inputs_len, 1);   % uncertainty on the not controllable inputs
  
  % Matrix used for the centroid's dynamic
  A_centroid = eye(states_len);                % state matrix
  B_centroid = [dt 0;
  0 dt;
  0 0 ];  % input matrix

  S_centroid = 1*eye(3);  % weight for states 
  R_centroid = 0.1*eye(2);  % weight for inputs
  Sf_centroid = 5*eye(3);               % weight for final state
  K_centroid = eye(2, 3);    % control matrix
else
  error('Model not implemented')
end

ground_th = 1/10*x0(3);    % distance from the ground to decelerate the agent

%% Control settings LQR
S = 1*eye(3);  % weight for states
R = 0.1*eye(inputs_len-1);  % weight for inputs
Sf = 5*eye(3);               % weight for final state
K = eye(inputs_len, 3);    % control matrix

%% Plots settings
marker_size = 10;
line_width = 2;
% colors
colors_vect = [[0 0.4470 0.7410]; [0.8500 0.3250 0.0980]; ...
               [0.9290 0.6940 0.1250]; [0.4940 0.1840 0.5560]; ...
               [0.4660 0.6740 0.1880]; [0.3010 0.7450 0.9330]; ...
               [0.6350 0.0780 0.1840]];
enable_video = 0; % 1 for enabling, 0 otherwise