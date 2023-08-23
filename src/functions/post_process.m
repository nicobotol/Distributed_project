function post_process_data = post_process(agents)

post_process_data = cell(length(agents),1);
for i=1:length(agents)
  post_process_data{i}.input_error = var(agents{i}.u_bar_store' -  agents{i}.u_store');
end

end