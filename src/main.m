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
parametric = 0;                                 % 0 for single simulation, 1 for parametric analysis, 2 for IK analysis
user_par = get_user_input(parametric);          % get user input
user_par.seed = 6;
par.parametric = parametric; 
par = parameters(variable_param, user_par, par);     

par.ag = [1 3 5 7 9 11 13]; 
par.ag_number = length(par.ag);
% Choose between par.parametric simulation or single simulation
if par.parametric == 0
  k = 1;                                      % number of the simulation
  % Comment if you want to have noise:
  % par = remove_noise(par);
  post_process_data = cell(1,1);
  [chute, post_process_data, true_centroid_store, par, w_store] = simulation(par, k, post_process_data);
elseif par.parametric == 1
  [chute, post_process_data, true_centroid_store, par, w_store] = parametric_analysis(par, variable_param, user_par);
elseif par.parametric == 2 % test with or without IK
  [result, chute] = comparison_IK(user_par, variable_param, par);
else
  error('Set the par.parametric variable to 0 or 1');
end

%% Plot
if(exist("j_fig") == 0)
  j_fig = 0;
end

if parametric == 1 || parametric == 0
  plot_chutes_trajectory(chute, true_centroid_store, j_fig, w_store, par, post_process_data, parametric);
    
  RMS_final_chute(chute, par, true_centroid_store);
end

disp('-------------------------------------------------------------------------')
disp('<strong>Simulation ended</strong>')
disp('<strong>-------------------------------------------------------------------------</strong>')

if par.parametric == 1
  % Save the post processed data in a variable in the workspace
  save(['post_process_data_', num2str(par.n_agents), '_agents.mat'], 'post_process_data')
  save(['par_', num2str(par.n_agents), '_agents.mat'], 'par')
end