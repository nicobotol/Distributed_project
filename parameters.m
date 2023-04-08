target = [0 0 0]';  % target point [x y z theta] [m m m rad]
x0 = [1 1 1]';      % initial state [m m m rad]
I = 1;              % inertia [kg*m^2]
dt = 0.01;          % time steep [s]
sim_t = 30;         % simulation time [s]

%% Simlation settings
rng(3);                   % random number generator seed
T = sim_t/dt;             % number of iterations [-]
t_vect = dt:dt:sim_t;     % [s]
states_len = length(x0);  % numer of states
inputs_len = 3;           % number of inputs
Qi_scale = 0.1;
Qi_bias = 0.5;
measure_len = 3;          % number of measurements
Ri_scale = 1;
Ri_bias = 0.5;

%% Control settings
k_p = 0.5; % proportional gain for the controller
k_i = 0.2;
k_d = 0.01;

%% Plots settings
marker_size = 10;
line_width = 2;
set(0,'DefaultFigureWindowStyle','docked');