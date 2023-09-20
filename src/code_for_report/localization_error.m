% close all

for i = 1:par.n_agents
  for j = 9:9 %par.n_agents % loop over the agents
    figure(); hold on;
    updated_model = chute{i}.loc_error{j}(1, :) == 2; % index when i update with the model
    updated_model_time = chute{i}.loc_error{j}(2, updated_model); % time when i update with the model
    updated_model_data = chute{i}.loc_error{j}(3:5, updated_model);
    updated_model_data_after_wls = chute{i}.loc_error_after_wls{j}(3:5, updated_model);
    updated_measure = chute{i}.loc_error{j}(1, :) == 1; % index when i update with the measure
    updated_measure_time = chute{i}.loc_error{j}(2, updated_measure); % time when i update with the measure
    updated_measure_data = chute{i}.loc_error{j}(3:5, updated_measure);
    updated_measure_data_after_wls = chute{i}.loc_error_after_wls{j}(3:5, updated_measure);
  
    all_update = [updated_model_time, updated_measure_time; updated_model_data, updated_measure_data];
    all_update_after_wls = [updated_model_time, updated_measure_time; updated_model_data_after_wls, updated_measure_data_after_wls];
    [~, order] = sort(all_update(1, :));
    all_update_sort = all_update(:, order);
    all_update_sort_after_wls = all_update_after_wls(:, order);
    plot(NaN, NaN, '-k', 'DisplayName', 'All localization')
    plot(NaN, NaN, '-xk', 'DisplayName', 'Model update')
    plot(NaN, NaN, '-ok', 'DisplayName', 'Measure update')
    plot(NaN, NaN, 'Color', color(1), 'DisplayName', 'x')
    plot(NaN, NaN, 'Color', color(2), 'DisplayName', 'y')
    plot(NaN, NaN, 'Color', color(3), 'DisplayName', 'z')
    for k=1:3 % loop over the three dimensions
      subplot(211)
      hold on
      plot(all_update_sort(1,:), all_update_sort(k+1,:), 'HandleVisibility', 'Off','Color', color(k))
      plot(chute{i}.loc_error{j}(2, updated_model_time), updated_model_data(k, :) , 'x', 'HandleVisibility', 'Off', 'Color', color(k))
      plot(chute{i}.loc_error{j}(2, updated_measure_time), updated_measure_data(k, :), 'o', 'HandleVisibility', 'Off', 'Color', color(k))
      
      subplot(212)
      hold on
      plot(all_update_sort(1,:), all_update_sort_after_wls(k+1,:), 'HandleVisibility', 'Off','Color', color(k))
      plot(chute{i}.loc_error{j}(2, updated_model_time), updated_model_data_after_wls(k, :) , 'x', 'HandleVisibility', 'Off', 'Color', color(k))
      plot(chute{i}.loc_error{j}(2, updated_measure_time), updated_measure_data_after_wls(k, :), 'o', 'HandleVisibility', 'Off', 'Color', color(k))
    end
    subplot(211)
    ylabel('Localization error before wls [m]')
    grid on
    box on
    subplot(212)
    ylabel('Localization error after wls [m]')
    xlabel('Iteration')
    grid on
    sgtitle(['Loc. error done by ', num2str(i), ' on ', num2str(j)]);
    legend();
    box on
  end
end