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
variable_param.prob_GPS = 3;                    % probability of getting GPS signal
variable_param.prob_connection = 3;             % probability of comunicating during the consensus
variable_param.prob_rel_measurement = 3;        % probability of measuring the relative position of the other chutes
par = parameters(variable_param);               % set the parameters of the simulation
% parametric = 0;                               % 1 for parametric analysis, 0 for single simulation
[par, parametric] = get_user_input(par);        % get user input

% Choose between parametric simulation or single simulation
if parametric == 1
  [chute, post_process_data, true_centroid_store, par, w_store] = parametric_analysis(par);
elseif parametric == 0
  % Set the value of the variable to be used in the simulation
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

if parametric == 1
  % Save the post processed data in a variable in the workspace
  save(['post_process_data_', num2str(par.n_agents), '_agents.mat'], 'post_process_data')
end