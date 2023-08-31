function [chute, post_process_data, true_centroid_store, par, w_store] = parametric_analysis()

variable_param.prob_GPS = 1;
variable_param.prob_conn = 1;
variable_param.prob_rel_measurement = 1;

par = parameters(variable_param);
% write here the number of simulations to be performed
number_simulations = size(par.prob_GPS_vec, 2) + size(par.prob_conn_vec, 2) + size(par.prob_rel_measurement_vec, 2); 
post_process_data = cell(number_simulations,1);

for k = 1:length(par.prob_GPS_vec)
  %% Load parameters
  par = set_parameters(k, variable_param, 'prob_GPS');
  [chute, post_process_data, true_centroid_store, par, w_store] = simulation(variable_param, par, k, post_process_data);
end

for k = 1:length(par.prob_conn_vec)
  %% Load parameters
  par = set_parameters(k, variable_param, 'prob_conn');
  [chute, post_process_data, true_centroid_store, par, w_store] = simulation(variable_param, par, k + length(par.prob_GPS_vec), post_process_data);
end

for k = 1:length(par.prob_rel_measurement_vec)
  %% Load parameters
  par = set_parameters(k, variable_param, 'prob_rel_measurement');
  [chute, post_process_data, true_centroid_store, par, w_store] = simulation(variable_param, par, k + length(par.prob_GPS_vec) + length(par.prob_conn_vec), post_process_data);
end

end