%% Constant parameters for the simualtion

%% Simulation parameters
dt = 0.01;          % time steep [s]
sim_t = 10;         % simulation time [s]
target = [0 0 0]';  % target point [x y z] [m m m]
x0 = [30 30 70;
      35 35 75;
      -10 -10 80]';   % initial state [x y z]'
Sigma = 1e0*eye(2); % std of the distribution

%% Parachute parameters
n_agents = 10;       % number of agents
position_range = 20;% range where the agents are deployed
Rc = 5;             % communication range of the robot
Rs = Rc/2;          % sensing range of the robot (i.e. where the robot can move at maximum to avoi collisions)
z_th = 1;           % height of the parachute
Delta = 1;          % agent dimension radius
vmax = 1;           % maximum velocity of the agent


%% Simulation settings
rng(5);                   % random number generator seed
T = sim_t/dt;             % number of iterations [-]
t_vect = dt:dt:sim_t;     % [s]
states_len = length(x0);  % numer of states
inputs_len = 3;           % number of inputs
Q_scale = 0.1;
Q_bias = 0.5;
measure_len = 3;          % number of measurements
R_GPS_scale = 1;
R_GPS_bias = 0.5;
n = size(x0, 2);          % number of parachudes
m = 10;                   % protocol to exchange to reach the consensus

%% Control settings LQR
S = eye(states_len);  % weight for states
R = eye(inputs_len);  % weight for inputs
Sf = S;               % weight for final state

%% Plots settings
marker_size = 10;
line_width = 2;
set(0,'DefaultFigureWindowStyle','docked');