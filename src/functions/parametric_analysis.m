function [chute, post_process_data, true_centroid_store, par, w_store] = parametric_analysis(par, variable_param)

% write here the number of simulations to be performed
number_simulations = size(par.prob_GPS_vec, 2) + size(par.prob_conn_vec, 2) + size(par.prob_rel_measurement_vec, 2); 
post_process_data = cell(number_simulations,1);

for k = 1:length(par.prob_GPS_vec)
  %% Load parameters
  par = set_parameters(k, variable_param, 'prob_GPS', user_par);
  [chute, post_process_data, true_centroid_store, par, w_store] = simulation(par, k, post_process_data);
end

for k = 1:length(par.prob_conn_vec)
  %% Load parameters
  par = set_parameters(k, variable_param, 'prob_connection', user_par);
  [chute, post_process_data, true_centroid_store, par, w_store] = simulation(par, k + length(par.prob_GPS_vec), post_process_data);
end

for k = 1:length(par.prob_rel_measurement_vec)
  %% Load parameters
  par = set_parameters(k, variable_param, 'prob_rel_measurement', user_par);
  [chute, post_process_data, true_centroid_store, par, w_store] = simulation(par, k + length(par.prob_GPS_vec) + length(par.prob_conn_vec), post_process_data);
end

end