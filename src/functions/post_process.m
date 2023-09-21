function post_process_data = post_process(agents, k, post_process_data, par)


  n_agents = par.n_agents;
  % This function computes the post processing data for the agents, i.e. the standard deviation of the input and localization error
  
  post_process_data{k}.agents = cell(length(agents), 1);
  input_error_sum = zeros(par.inputs_len, 1);
  localization_error_sum = zeros(par.states_len, 1);
  
  for i=1:length(agents)
    input_error = std(agents{i}.u_bar_store' - agents{i}.u_store')';
    post_process_data{k}.agents{i}.input_error = input_error;
    input_error_sum = input_error_sum + input_error; 
    mean_loc_error_other = 0;
    mean_loc_error_other_after_wls = 0;
    seen_agent = 0;
    seen_agent_after_wls = 0;

    % len_x_real_store = size(agents{i}.x_real_store, 2);
    % localization_error = std(agents{i}.x_real_store(:, 50:end)' -  agents{i}.x_store(:, 50:len_x_real_store)')';
    localization_error = std(agents{i}.x_real_store' -  agents{i}.x_store')';
    post_process_data{k}.agents{i}.localization_error = localization_error;
    localization_error_sum = localization_error_sum + localization_error;

    % Error done by i on the localization of the others
    agents{i}.agents{i}.updated_model_time = cell(n_agents, 1);
    agents{i}.agents{i}.updated_model_data = cell(n_agents, 1);
    agents{i}.updated_model_data_after_wls = cell(n_agents, 1);
    agents{i}.agents{i}.updated_measure_time = cell(n_agents, 1);
    agents{i}.agents{i}.updated_measure_data = cell(n_agents, 1);
    agents{i}.updated_measure_data_after_wls = cell(n_agents, 1);
    agents{i}.all_update_sort = cell(n_agents, 1);
    agents{i}.all_update_sort_after_wls = cell(n_agents, 1);
    all_update_sort = [];
    all_update_sort_after_wls = [];
    post_process_data{k}.agents{i}.loc_error_std = cell(n_agents, 1);
    post_process_data{k}.agents{i}.loc_error_mean = cell(n_agents, 1);
    post_process_data{k}.agents{i}.loc_error_std_after_wls = cell(n_agents, 1);
    post_process_data{k}.agents{i}.loc_error_mean_after_wls = cell(n_agents, 1);

    for j = 1:n_agents %par.n_agents % loop over the agents
      % before the WLS
      agents{i}.updated_measure{j} = agents{i}.loc_error{j}(1, :) == 1; % index when i update with the measure
      agents{i}.updated_measure_time{j} = agents{i}.loc_error{j}(2, agents{i}.updated_measure{j}); % time when i update with the measure
      agents{i}.updated_measure_data{j} = agents{i}.loc_error{j}(3:5, agents{i}.updated_measure{j});
      agents{i}.updated_model{j} = agents{i}.loc_error{j}(1, :) == 2; % index when i update with the model
      agents{i}.updated_model_time{j} = agents{i}.loc_error{j}(2, agents{i}.updated_model{j}); % time when i update with the model
      agents{i}.updated_model_data{j} = agents{i}.loc_error{j}(3:5, agents{i}.updated_model{j}); % localization error when update with the model
      agents{i}.all_updated{j} = [agents{i}.updated_model_time{j}, agents{i}.updated_measure_time{j}; agents{i}.updated_model_data{j}, agents{i}.updated_measure_data{j}]; % all the localization before the wls
      
      agents{i}.after_wls{j} = agents{i}.loc_error_after_wls{j}(1, :) == 1; % index when i updates j with the WLS
      agents{i}.after_wls_data{j} = agents{i}.loc_error_after_wls{j}(3:5,  agents{i}.after_wls{j});
      
      if ~isempty(agents{i}.all_updated{j})
        % sort the data
        [~, order] = sort(agents{i}.all_updated{j}(1, :));
        agents{i}.all_update_sort{j} = agents{i}.all_updated{j}(:, order);
        
        % compute mean and std
        agents{i}.loc_error_std{j} = std(agents{i}.all_update_sort{j}(2:4, :)');
        agents{i}.loc_error_mean{j} = mean(agents{i}.all_update_sort{j}(2:4, :)');

        % increase the counter of updated agents
        seen_agent = seen_agent + 1;
      else
        agents{i}.loc_error_std{j} = zeros(3,1);
        agents{i}.loc_error_mean{j} = zeros(3,1);
      end

      if ~isempty(agents{i}.after_wls_data{j}())
        
        agents{i}.loc_error_std_after_wls{j} = std(agents{i}.after_wls_data{j}');
        agents{i}.loc_error_mean_after_wls{j} = mean(agents{i}.after_wls_data{j}');
        seen_agent_after_wls = seen_agent_after_wls + 1;
      else
        agents{i}.loc_error_std_after_wls{j} =  zeros(3,1);
        agents{i}.loc_error_mean_after_wls{j} =  zeros(3,1);
        
      end

     mean_loc_error_other_after_wls = mean_loc_error_other_after_wls + agents{i}.loc_error_std_after_wls{j};

     post_process_data{k}.agents{i}.loc_error_std{j} = agents{i}.loc_error_std{j};
     post_process_data{k}.agents{i}.loc_error_mean{j} = agents{i}.loc_error_mean{j};
     post_process_data{k}.agents{i}.loc_error_std_after_wls{j} = agents{i}.loc_error_std_after_wls{j};
     post_process_data{k}.agents{i}.loc_error_mean_after_wls{j} = agents{i}.loc_error_mean_after_wls{j};

    end

    post_process_data{k}.agents{i}.mean_loc_error_other = mean_loc_error_other/seen_agent;
    post_process_data{k}.agents{i}.mean_loc_error_other_after_wls = mean_loc_error_other_after_wls/seen_agent_after_wls;
  end

  % means of the standard deviations for every agent in x and u
  post_process_data{k}.mean_std_u = input_error_sum/length(agents);
  post_process_data{k}.mean_std_x = localization_error_sum/length(agents);

end