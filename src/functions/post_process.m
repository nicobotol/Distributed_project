function post_process_data = post_process(agents, k, post_process_data, par)


  n_agents = par.n_agents;
  % This function computes the post processing data for the agents, i.e. the standard deviation of the input and localization error
  
  post_process_data{k}.agents = cell(length(agents), 1);
  input_error_sum = zeros(par.inputs_len, 1);
  localization_error_sum = zeros(par.states_len, 1);
  
  for i=1:length(agents)
    mean_loc_error_other = 0;
    mean_loc_error_other_after_wls = 0;
    std_loc_error_other = 0;
    std_loc_error_other_after_wls = 0;
    seen_agent = 0;
    seen_agent_after_wls = 0;

    % Error done by i on the localization of the others
    agents{i}.post_process_data{k}.agents{i}.updated_model_time = cell(n_agents, 1);
    agents{i}.post_process_data{k}.agents{i}.updated_model_data = cell(n_agents, 1);
    post_process_data{k}.agents{i}.updated_model_data_after_wls = cell(n_agents, 1);
    agents{i}.agents{i}.updated_measure_time = cell(n_agents, 1);
    agents{i}.agents{i}.post_process_data{k}.updated_measure_data = cell(n_agents, 1);
    agents{i}.post_process_data{k}.updated_measure_data_after_wls = cell(n_agents, 1);
    agents{i}.all_update_sort = cell(n_agents, 1);
    agents{i}.all_update_sort_after_wls = cell(n_agents, 1);
    all_update_sort = [];
    all_update_sort_after_wls = [];
    post_process_data{k}.agents{i}.loc_error_mean = cell(n_agents, 1);
    post_process_data{k}.agents{i}.loc_error_mean_after_wls = cell(n_agents, 1);
    post_process_data{k}.agents{i}.loc_error_std = cell(n_agents, 1);
    post_process_data{k}.agents{i}.loc_error_std_after_wls = cell(n_agents, 1);

    for j = 1:n_agents %par.n_agents % loop over the agents
%   _____      _                  _   _             
%  | ____|_  _| |_ _ __ __ _  ___| |_(_) ___  _ __  
%  |  _| \ \/ / __| '__/ _` |/ __| __| |/ _ \| '_ \ 
%  | |___ >  <| |_| | | (_| | (__| |_| | (_) | | | |
%  |_____/_/\_\\__|_|  \__,_|\___|\__|_|\___/|_| |_|
                                                  
      % before the WLS
      post_process_data{k}.agents{i}.updated_measure{j} = agents{i}.loc_error{j}(1, :) == 1; % index when i update with the measure
      post_process_data{k}.agents{i}.updated_measure_time{j} = agents{i}.loc_error{j}(2, post_process_data{k}.agents{i}.updated_measure{j}); % time when i update with the measure
      post_process_data{k}.agents{i}.updated_measure_data{j} = agents{i}.loc_error{j}(3:5, post_process_data{k}.agents{i}.updated_measure{j});
      post_process_data{k}.agents{i}.updated_model{j} = agents{i}.loc_error{j}(1, :) == 2; % index when i update with the model
      post_process_data{k}.agents{i}.updated_model_time{j} = agents{i}.loc_error{j}(2, post_process_data{k}.agents{i}.updated_model{j}); % time when i update with the model
      post_process_data{k}.agents{i}.updated_model_data{j} = agents{i}.loc_error{j}(3:5, post_process_data{k}.agents{i}.updated_model{j}); % localization error when update with the model
      post_process_data{k}.agents{i}.all_updated{j} = [post_process_data{k}.agents{i}.updated_model_time{j}, post_process_data{k}.agents{i}.updated_measure_time{j}; post_process_data{k}.agents{i}.updated_model_data{j}, post_process_data{k}.agents{i}.updated_measure_data{j}]; % all the localization before the wls
      
      % after the WLS
      agents{i}.after_wls{j} = agents{i}.loc_error_after_wls{j}(1, :) == 1; % index when i updates j with the WLS
      agents{i}.after_wls_data{j} = agents{i}.loc_error_after_wls{j}(3:5,  agents{i}.after_wls{j});

%   ____           _     ____                              _             
%  |  _ \ ___  ___| |_  |  _ \ _ __ ___   ___ ___  ___ ___(_)_ __   __ _ 
%  | |_) / _ \/ __| __| | |_) | '__/ _ \ / __/ _ \/ __/ __| | '_ \ / _` |
%  |  __/ (_) \__ \ |_  |  __/| | | (_) | (_|  __/\__ \__ \ | | | | (_| |
%  |_|   \___/|___/\__| |_|   |_|  \___/ \___\___||___/___/_|_| |_|\__, |
%                                                                  |___/ 
      % Before the WLS
      if ~isempty(post_process_data{k}.agents{i}.all_updated{j})
        % sort the data
        [~, order] = sort(post_process_data{k}.agents{i}.all_updated{j}(1, :));
        post_process_data{k}.agents{i}.all_update_sort{j} = post_process_data{k}.agents{i}.all_updated{j}(:, order);
        
        % compute mean and the std of the localization error
        post_process_data{k}.agents{i}.loc_error_mean{j} = mean(post_process_data{k}.agents{i}.all_update_sort{j}(2:4, :)'); % mean of the localization error commited by i on j
        post_process_data{k}.agents{i}.loc_error_std{j} = std(post_process_data{k}.agents{i}.all_update_sort{j}(2:4, :)'); % std of the localization error commited by i on j

        if j ~= i
          % increase the counter of updated agents
          seen_agent = seen_agent + 1;
          mean_loc_error_other = mean_loc_error_other + post_process_data{k}.agents{i}.loc_error_mean{j};
          std_loc_error_other = std_loc_error_other + post_process_data{k}.agents{i}.loc_error_std{j};
        end
      else
        post_process_data{k}.agents{i}.loc_error_mean{j} = [NaN, NaN, NaN]';
        post_process_data{k}.agents{i}.loc_error_std{j} = [NaN, NaN, NaN]';
      end


      % After the WLS
      if ~isempty(agents{i}.after_wls_data{j}())
        % compute the mean and the std of the localization error after the wls
        post_process_data{k}.agents{i}.loc_error_mean_after_wls{j} = mean(agents{i}.after_wls_data{j}');
        post_process_data{k}.agents{i}.loc_error_std_after_wls{j} = std(agents{i}.after_wls_data{j}');

        if j ~= i 
          seen_agent_after_wls = seen_agent_after_wls + 1;
          mean_loc_error_other_after_wls = mean_loc_error_other_after_wls + post_process_data{k}.agents{i}.loc_error_mean_after_wls{j};
          std_loc_error_other_after_wls = std_loc_error_other_after_wls + post_process_data{k}.agents{i}.loc_error_std_after_wls{j};
        end
      else
        post_process_data{k}.agents{i}.loc_error_mean_after_wls{j} =  [NaN, NaN, NaN]';
        post_process_data{k}.agents{i}.loc_error_std_after_wls{j} = [NaN, NaN, NaN]';
      end

    end

    post_process_data{k}.agents{i}.mean_loc_error_other = mean_loc_error_other/seen_agent; % mean of the localization error done by i on all the other agetns (apart from itself)
    post_process_data{k}.agents{i}.mean_loc_error_other_after_wls = mean_loc_error_other_after_wls/seen_agent_after_wls;
    post_process_data{k}.agents{i}.std_loc_error_other = std_loc_error_other/seen_agent; % mean of the std of the localization error done by i on all the others
    post_process_data{k}.agents{i}.std_loc_error_other_after_wls = std_loc_error_other_after_wls/seen_agent_after_wls;
  end
end