close all
clc
figure()
hold on
idx_plot = 1; % agent i that i want to plot
jdx_plot = 4; % agent j that i want to plot
plot(NaN, NaN, '-', 'Displayname', 'all')
plot(NaN, NaN, 'o', 'Displayname', 'model')
plot(NaN, NaN, 'x', 'Displayname', 'measure')
for k=1:3
  time = post_process_data{1}.agents{idx_plot}.all_update_sort{jdx_plot}(1,:);
  time_model =  post_process_data{1}.agents{idx_plot}.updated_model_time{jdx_plot};
  time_measure =  post_process_data{1}.agents{idx_plot}.updated_measure_time{jdx_plot};
  all_data = post_process_data{1}.agents{idx_plot}.all_update_sort{jdx_plot}(k+1, :);
  measure_data = post_process_data{1}.agents{idx_plot}.updated_measure_data{jdx_plot}(k, :);
  model_data = post_process_data{1}.agents{idx_plot}.updated_model_data{jdx_plot}(k, :);
  plot(time, all_data, 'Color', color(k), 'HandleVisibility', 'off'); % 2=x, 3=y, 4=z
  plot(time_model, model_data, 'o', 'Color', color(k), 'HandleVisibility', 'off')
  plot(time_measure, measure_data, 'x', 'Color', color(k), 'HandleVisibility', 'off')
end
legend()
grid on
box on
xlabel('Iteration')
ylabel('Error')
title(['Localization done by ', num2str(idx_plot), ' on ', num2str(jdx_plot)])