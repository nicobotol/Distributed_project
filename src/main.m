clc
close all
clear

format short

if ispc
  path = 'functions\';
else
  path = 'functions/';
end

addpath(path);
initialize_environment;

variable_param.prob_GPS = 1;                    % probability of getting GPS signal
variable_param.prob_connection = 1;             % probability of comunicating during the consensus
variable_param.prob_rel_measurement = 1;        % probability of measuring the relative position of the other chutes
parametric = 0;                                 % 1 for parametric analysis, 0 for single simulation
user_par = get_user_input();                    % get user input
par = parameters(variable_param, user_par);      

% Choose between parametric simulation or single simulation
if parametric == 1
  [chute, post_process_data, true_centroid_store, par, w_store] = parametric_analysis(par, variable_param, user_par);
elseif parametric == 0
  k = 1;                                      % number of the simulation
  post_process_data = cell(1,1);
  [chute, post_process_data, true_centroid_store, par, w_store] = simulation(par, k, post_process_data);
else
  error('Set the parametric variable to 0 or 1');
end

%% Plot
if(exist("j_fig") == 0)
  j_fig = 0;
end
plot_chutes_trajectory(chute, true_centroid_store, j_fig, w_store, par, post_process_data, parametric);

RMS_final_chute(chute, par);

disp('-------------------------------------------------------------------------')
disp('<strong>Simulation ended</strong>')
disp('<strong>-------------------------------------------------------------------------</strong>')

if parametric == 1
  % Save the post processed data in a variable in the workspace
  save(['post_process_data_', num2str(par.n_agents), '_agents.mat'], 'post_process_data')
end