function post_process_data = post_process(agents)

% This function computes the post processing data for the agents, i.e. the standard deviation of the input and localization error

post_process_data = cell(length(agents),1);
for i=1:length(agents)
  post_process_data{i}.input_error = std(agents{i}.u_bar_store' -  agents{i}.u_store');
  post_process_data{i}.localization_error = std(agents{i}.x_real_store' -  agents{i}.x_store');
end

end