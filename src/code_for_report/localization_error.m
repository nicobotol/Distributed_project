close all

max_i = 1;
max_j = 1;
n_agents = par.n_agents;
marker_size = par.marker_size;
line_width = par.line_width;
font_size = par.font_size;
for i = 1:n_agents
  chute{i}.updated_model_time = cell(n_agents, 1);
  chute{i}.updated_model_data = cell(n_agents, 1);
  chute{i}.updated_model_data_after_wls = cell(n_agents, 1);
  chute{i}.updated_measure_time = cell(n_agents, 1);
  chute{i}.updated_measure_data = cell(n_agents, 1);
  chute{i}.updated_measure_data_after_wls = cell(n_agents, 1);
  chute{i}.all_update_sort = cell(n_agents, 1);
  chute{i}.all_update_sort_after_wls = cell(n_agents, 1);
  all_update_sort = [];
  all_update_sort_after_wls = [];
  for j = 1:n_agents %par.n_agents % loop over the agents
    updated_measure = chute{i}.loc_error{j}(1, :) == 1; % index when i update with the measure
    updated_measure_time = chute{i}.loc_error{j}(2, updated_measure); % time when i update with the measure
    updated_measure_data = chute{i}.loc_error{j}(3:5, updated_measure);
    updated_measure_data_after_wls = chute{i}.loc_error_after_wls{j}(3:5, updated_measure);

    updated_model = chute{i}.loc_error{j}(1, :) == 2; % index when i update with the model
    updated_model_time = chute{i}.loc_error{j}(2, updated_model); % time when i update with the model
    updated_model_data = chute{i}.loc_error{j}(3:5, updated_model); % localization error when update with the model
    updated_model_data_after_wls = chute{i}.loc_error_after_wls{j}(3:5, updated_model); % localization error when update with the model, after the WLS

    chute{i}.updated_model_time{j} = updated_model_time;
    chute{i}.updated_model_data{j} = updated_model_data;
    chute{i}.updated_model_data_after_wls{j} = updated_model_data_after_wls;
    chute{i}.updated_measure_time{j} = updated_measure_time;
    chute{i}.updated_measure_data{j} = updated_measure_data;
    chute{i}.updated_measure_data_after_wls{j} = updated_measure_data_after_wls;
    
    all_update = [updated_model_time, updated_measure_time; updated_model_data, updated_measure_data];
    all_update_after_wls = [updated_model_time, updated_measure_time; updated_model_data_after_wls, updated_measure_data_after_wls];
    if ~isempty(all_update)
      [~, order] = sort(all_update(1, :));
      chute{i}.all_update_sort{j} = all_update(:, order);
      chute{i}.all_update_sort_after_wls{j} = all_update_after_wls(:, order);
      
      chute{i}.loc_error_std{j} = std(chute{i}.all_update_sort{j}(2:4, :)');
      chute{i}.loc_error_std_after_wls{j} = std(chute{i}.all_update_sort_after_wls{j}(2:4, :)');
      chute{i}.loc_error_mean{j} = mean(chute{i}.all_update_sort{j}(2:4, :)');
      chute{i}.loc_error_mean_after_wls{j} = mean(chute{i}.all_update_sort_after_wls{j}(2:4, :)');
    end
  end
end

for i = 1:max_i
  for j = 1:max_j %par.n_agents % loop over the agents
    figure(); 
    plot(NaN, NaN, '-k', 'DisplayName', 'All localization')
    plot(NaN, NaN, '-xk', 'DisplayName', 'Model update')
    plot(NaN, NaN, '-ok', 'DisplayName', 'Measure update')
    plot(NaN, NaN, '--k', 'DisplayName', 'Mean')
    plot(NaN, NaN, 'Color', color(1), 'DisplayName', 'x')
    plot(NaN, NaN, 'Color', color(2), 'DisplayName', 'y')
    plot(NaN, NaN, 'Color', color(3), 'DisplayName', 'z')
    for k=1:3 % loop over the three dimensions
      subplot(211)
      hold on
      plot(chute{i}.all_update_sort{j}(1,:), chute{i}.all_update_sort{j}(k+1,:), 'HandleVisibility', 'Off','Color', color(k));
      plot(chute{i}.loc_error{j}(2, chute{i}.updated_model_time{j}), chute{i}.updated_model_data{j}(k, :) , 'x', 'HandleVisibility', 'Off', 'Color', color(k));
      plot(chute{i}.loc_error{j}(2, chute{i}.updated_measure_time{j}), chute{i}.updated_measure_data{j}(k, :), 'o', 'HandleVisibility', 'Off', 'Color', color(k));
      yline(chute{i}.loc_error_mean{j}(k), '--', 'Color', color(k));
      
      subplot(212)
      hold on
      plot(chute{i}.all_update_sort{j}(1,:), chute{i}.all_update_sort_after_wls{j}(k+1,:), 'HandleVisibility', 'Off','Color', color(k))
      plot(chute{i}.loc_error{j}(2, chute{i}.updated_model_time{j}), chute{i}.updated_model_data_after_wls{j}(k, :) , 'x', 'HandleVisibility', 'Off', 'Color', color(k))
      plot(chute{i}.loc_error{j}(2, chute{i}.updated_measure_time{j}), chute{i}.updated_measure_data_after_wls{j}(k, :), 'o', 'HandleVisibility', 'Off', 'Color', color(k))
      yline(chute{i}.loc_error_mean_after_wls{j}(k), '--', 'Color', color(k))
    end
    subplot(211)
    ylabel('Localization error before wls [m]')
    grid on
    box on
    subplot(212)
    legend('Location', 'best');
    ylabel('Localization error after wls [m]')
    xlabel('Iteration')
    grid on
    sgtitle(['Loc. error done by ', num2str(i), ' on ', num2str(j)]);
    box on
  end
end

for i = 1:n_agents
  figure(); hold on;
  plot(NaN, NaN, '-', 'Color', color(1), 'DisplayName', 'Before', 'MarkerSize', marker_size, 'LineWidth', line_width)
  plot(NaN, NaN, '-', 'Color', color(2), 'DisplayName', 'After', 'MarkerSize', marker_size, 'LineWidth', line_width)
  plot(NaN, NaN, 'xk', 'DisplayName', 'x', 'MarkerSize', marker_size, 'LineWidth', line_width)
  plot(NaN, NaN, 'ok', 'DisplayName', 'y', 'MarkerSize', marker_size, 'LineWidth', line_width)
  plot(NaN, NaN, 'sk', 'DisplayName', 'z', 'MarkerSize', marker_size, 'LineWidth', line_width)
  for j = 1:n_agents
    plot(j, chute{i}.loc_error_std{j}(1), 'x', 'Color', color(1), 'HandleVisibility', 'off', 'MarkerSize', marker_size, 'LineWidth', line_width);
    plot(j, chute{i}.loc_error_std{j}(2), 'o', 'Color', color(1), 'HandleVisibility', 'off', 'MarkerSize', marker_size, 'LineWidth', line_width);
    plot(j, chute{i}.loc_error_std{j}(3), 's', 'Color', color(1), 'HandleVisibility', 'off', 'MarkerSize', marker_size, 'LineWidth', line_width);
    plot(j, chute{i}.loc_error_std_after_wls{j}(1), 'x', 'Color', color(2), 'HandleVisibility', 'off', 'MarkerSize', marker_size, 'LineWidth', line_width);
    plot(j, chute{i}.loc_error_std_after_wls{j}(2), 'o', 'Color', color(2), 'HandleVisibility', 'off', 'MarkerSize', marker_size, 'LineWidth', line_width);
    plot(j, chute{i}.loc_error_std_after_wls{j}(3), 's', 'Color', color(2), 'HandleVisibility', 'off', 'MarkerSize', marker_size, 'LineWidth', line_width);
  end
  xlabel('Localized agent', 'FontSize', font_size)
  ylabel('Error in localization', 'FontSize', font_size)
  title(['Localization done by agent ', num2str(i)], 'FontSize', font_size)
  grid on
  box on
  legend('location', 'best', 'FontSize', font_size)
  xlim([0 n_agents+1])
end