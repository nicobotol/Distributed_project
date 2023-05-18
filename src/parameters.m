%% Constant parameters for the simualtion

%% Simulation parameters
dt = 0.01;          % time steep [s]
sim_t = 10;         % simulation time [s]
target = [0 0 0]';  % target point [x y z] [m m m]
x0 = [30 30 70;
      35 35 75;
      -10 -10 80]';     % initial state [x y z]'
Sigma = 1e0*eye(2);     % std of the distribution used for navigation

%% Parachute parameters
n_agents = 6;       % number of agents
position_range = 10;% range where the agents are deployed
Rc = 6;             % communication range of the robot
Rs = Rc/2;          % sensing range of the robot (i.e. where the robot can move at maximum to avoi collisions)
z_th = 1;           % height of the parachute
Delta = 0.1;          % agent dimension radius
vmax = 10;           % maximum velocity of the agent

%% Simulation settings
rng(5);                   % random number generator seed
T = sim_t/dt;             % number of iterations [-]
t_vect = dt:dt:sim_t;     % [s]
states_len = length(x0);  % numer of states
inputs_len = 3;           % number of inputs
Q_scale = 0.1;
Q_bias = 0.5;
measure_len = 3;          % number of measurements
R_GPS_scale = 0.2;
R_GPS_bias = 0.5;
L_scale = 0.1; 
L_bias = 0.5;
n = n_agents;             % number of parachudes
m = 100;                   % protocol to exchange to reach the consensus
P_est_init = 1000;         % random initial position covariance value
P_est_threshold = norm(P_est_init*eye(states_len, states_len)); % threshold for the covariance matrix to ignore far agents

%% Dynamics parameters
A = eye(states_len);                % state matrix
B = eye(states_len,inputs_len)*dt;  % input matrix
G = 0*eye(states_len, states_len);  % noise matrix

%% Control settings LQR
S = eye(states_len);  % weight for states
R = eye(inputs_len);  % weight for inputs
Sf = S;               % weight for final state
K = eye(inputs_len, states_len);    % control matrix

%% Plots settings
marker_size = 10;
line_width = 2;
set(0,'DefaultFigureWindowStyle','docked');