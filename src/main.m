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
parametric = 0;                               % 1 for parametric analysis, 0 for single simulation

% Choose between parametric simulation or single simulation
if parametric == 1
  [chute, post_process_data, true_centroid_store, par, w_store] = parametric_analysis();
elseif parametric == 0
  % Set the value of the variable to be used in the simulation
  variable_param.prob_GPS = 4;                % probability of getting GPS signal
  variable_param.prob_conn = 4;               % probability of comunicating during the consensus
  variable_param.prob_rel_measurement = 4;    % probability of measuring the relative position of the other chutes
  par = parameters(variable_param);
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