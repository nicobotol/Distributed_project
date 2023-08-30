function post_process_data = post_process(agents, k, post_process_data, par)

  % This function computes the post processing data for the agents, i.e. the standard deviation of the input and localization error
  
  post_process_data{k}.agents = cell(length(agents), 1);
  input_error_sum = zeros(par.inputs_len, 1);
  localization_error_sum = zeros(par.states_len, 1);
  
  for i=1:length(agents)
    input_error = std(agents{i}.u_bar_store' - agents{i}.u_store')';
    post_process_data{k}.agents{i}.input_error = input_error;
    input_error_sum = input_error_sum + input_error; 

    localization_error = std(agents{i}.x_real_store' -  agents{i}.x_store(:, 1:size(agents{i}.x_real_store, 2))')';
    post_process_data{k}.agents{i}.localization_error = localization_error;
    localization_error_sum = localization_error_sum + localization_error;
  end

  % means of the standard deviations for every agent in x and u
  post_process_data{k}.mean_std_u = input_error_sum/length(agents);
  post_process_data{k}.mean_std_x = localization_error_sum/length(agents);

end