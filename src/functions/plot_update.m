close all
clc

line_width = par.line_width*0.5;
marker_size = par.marker_size*0.5;
font_size =  par.font_size;

for idx_plot = 1:1
  jdx_plot = idx_plot;
fig_self_update = figure('Color', 'w');
hold on
% idx_plot = 1; % agent i that i want to plot
% jdx_plot = 1; % agent j that i want to plot
kdx_plot = 1;
plot(NaN, NaN, '-', 'Displayname', 'x','Color', color(1), 'LineWidth', line_width)
plot(NaN, NaN, '-', 'Displayname', 'y','Color', color(2), 'LineWidth', line_width)
plot(NaN, NaN, '-', 'Displayname', 'z','Color', color(3), 'LineWidth', line_width)
plot(NaN, NaN, 'o', 'Displayname', 'model', 'LineWidth', line_width, 'MarkerSize',marker_size)
plot(NaN, NaN, 'x', 'Displayname', 'measure', 'LineWidth', line_width, 'MarkerSize',marker_size)
yyaxis left 
ylim([-10 12])
ylabel('Error in x and y [m]', 'FontSize',font_size)
for kk=1:3
  time = post_process_data{kdx_plot}.agents{idx_plot}.all_update_sort{jdx_plot}(1,:);
  time_model =  post_process_data{kdx_plot}.agents{idx_plot}.updated_model_time{jdx_plot};
  time_measure =  post_process_data{kdx_plot}.agents{idx_plot}.updated_measure_time{jdx_plot};
  all_data = post_process_data{kdx_plot}.agents{idx_plot}.all_update_sort{jdx_plot}(kk+1, :);
  measure_data = post_process_data{kdx_plot}.agents{idx_plot}.updated_measure_data{jdx_plot}(kk, :);
  model_data = post_process_data{kdx_plot}.agents{idx_plot}.updated_model_data{jdx_plot}(kk, :);
  if kk == 3
    yyaxis right
  end
  plot(time, all_data,'-', 'Color', color(kk), 'HandleVisibility', 'off', 'LineWidth', line_width, 'MarkerSize',marker_size); % 2=x, 3=y, 4=z
  plot(time_model, model_data, 'o', 'Color', color(kk), 'HandleVisibility', 'off', 'LineWidth', line_width, 'MarkerSize',marker_size)
  plot(time_measure, measure_data, 'x', 'Color', color(kk), 'HandleVisibility', 'off', 'LineWidth', line_width, 'MarkerSize',marker_size)
end
xline(chute{idx_plot}.t_falling-1, '--', 'Color', color(4),'LineWidth', line_width*2, 'HandleVisibility', 'off')
text(chute{idx_plot}.t_falling-1,55,'$\leftarrow$ End of free fall','FontSize',font_size)
xline(154, '--', 'Color', color(11), 'LineWidth', line_width*2, 'HandleVisibility', 'off')
text(154,-17,'$\leftarrow$ Landing', 'FontSize',font_size)
legend('FontSize',font_size,'NumColumns',2)
grid on
box on
xlabel('Iteration', 'FontSize',font_size)
ylabel('Error in z [m]', 'Color',color(3), 'FontSize',font_size)
ylim([-20 60])
set(gca, 'FontSize', font_size)
ax = gca;
ax.YAxis(1).Color = 'k';
ax.YAxis(2).Color = color(3);
title('Self-localization error, parachute 1', 'FontSize',font_size)
if par.enable_export == 1
    export_figure(fig_self_update, 'fig_self_update.eps', 'images\');
end
end