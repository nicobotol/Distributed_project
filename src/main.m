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
variable_param.prob_GPS = 1;  % prob comm
variable_param.prob_conn = 1;
variable_param.prob_rel_measurement = 1;

par = parameters(variable_param);
number_simulations = size(par.prob_GPS_vec, 1); % write here the number of simulations to be performed
post_process_data = cell(number_simulations,1);

for k = 1:length(par.prob_GPS_vec)
  %% Load parameters
  par = set_parameters(k, variable_param, 'prob_GPS');
  post_process_data = simulation(variable_param, par, k); % run the diferent simulation
end

for k = 1:length(par.prob_conn_vec)
  %% Load parameters
  par = set_parameters(k, 'prob_conn');
  post_process_data = simulation(variable_param, par, k + length(par.prob_GPS_vec)); % run the diferent simulation
end

for k = 1:length(par.prob_rel_measurement_vec)
  %% Load parameters
  par = set_parameters(k, 'prob_rel_measurement');
  post_process_data = simulation(variable_param, par, k + length(par.prob_GPS_vec) + length(prob_rel_measurement)); % run the diferent simulation
end

%% Plot
if(exist("j_fig") == 0)
  j_fig = 0;
end
plot_chutes_trajectory(chute, true_centroid_store, j_fig, w_store, par, post_process_data);

RMS_final_chute(chute, par);