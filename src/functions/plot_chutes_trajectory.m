function [] = plot_chutes_trajectory(agents,true_centroid_store, j_fig, w_store, par, post_process_data, parametric)

  marker_size = par.marker_size;
  dt = par.dt;
  target = par.target;
  mdl = par.mdl;
  enable_export = par.enable_export;
  n_agents = par.n_agents;
  x0 = par.x0;
  position_range = par.position_range;
  line_width = par.line_width;

  % if there is only one simulation plot the trajectory, the states, the inputs and the falling velocity
  if parametric == 0
    %% 3D trajectories
    figure('Name', '3D trajectory','NumberTitle','off', 'Color', 'w'); clf;
    hold all
    for i=1:n_agents
      plot3(agents{i}.x_real_store(1,:), agents{i}.x_real_store(2, :), agents{i}.x_real_store(3, :),'DisplayName', ['Agent ', num2str(i)])
    end
    plot3(target(1), target(2), target(3), 'o', 'MarkerSize', marker_size,'DisplayName', 'TARGET');
    plot3(true_centroid_store(1, :), true_centroid_store(2,:), true_centroid_store(3, :), 'r--', 'DisplayName', 'Centroid', 'LineWidth', 2)
    xlabel('x [m] ')
    ylabel('y [m]')
    zlabel('z [m]')
    legend('Location', 'bestoutside')
    grid on
    xlim([-abs(2*x0(1)+0.6*position_range), abs(x0(1)+0.6*position_range)])
    ylim([-abs(x0(2)+0.6*position_range), abs(x0(2)+0.6*position_range)])
    zlim([0, x0(3) + 0.6*position_range])
    axis equal

    %% Vertical displacement
    figure('Name', 'Vert. disp.','NumberTitle','off', 'Color', 'w'); clf;
    hold all
    for i=1:n_agents
      plot(agents{i}.x_store(2,:), agents{i}.x_store(3, :),'o','DisplayName', ['Agent ', num2str(i)])
    end
    plot(target(2), target(3), 'o', 'MarkerSize', marker_size,'DisplayName', 'TARGET');
    plot(true_centroid_store(2,:), true_centroid_store(3, :), 'r--', 'DisplayName', 'Centroid')
    xlabel('y [m]')
    ylabel('z [m]')
    legend('Location', 'bestoutside')
    grid on

    j_fig = 0;
    %% Inputs  and trajectories
    for i=1:n_agents
      j_fig = j_fig+1;
      figure('Name', ['Chute', num2str(j_fig)],'NumberTitle','off', 'Color', 'w'); clf;
      subplot(211);  hold all
      plot(agents{i}.x_store(1,1:end),'--','DisplayName', 'x', 'color', 'b')
      plot(agents{i}.x_real_store(1,1:end),'DisplayName','x real','color','b')
      plot(agents{i}.x_store(2,1:end),'--','DisplayName', 'y', 'color', 'k')
      plot(agents{i}.x_real_store(2,1:end),'DisplayName','y real','color','k')
      plot(agents{i}.x_store(3,1:end),'--','DisplayName', 'z', 'color', 'g')
      ylabel('[m]')
      plot(agents{i}.x_real_store(3,1:end),'DisplayName','z real','color','g')
      if mdl == 2
        yyaxis right
        plot(agents{i}.x_store(4,1:end),'--','DisplayName', '$\theta$', 'color', 'r')
        hold on
        plot(agents{i}.x_real_store(4,1:end),'-','DisplayName','$\theta$ real','color','r')
        ylabel('[rad]', 'color', 'r')
        set(gca, 'YColor', 'r')
      end
      title('State')
      legend('Location', 'eastoutside')
      grid on
      box on
      switch mdl
      case 1 % linear dynamic
        u_1 = '$v_x$';
        u_2 = '$v_y$';
        u_3 = '$v_z$';
      case 2 % unicycle dynamic
        u_1 = '$V$';
        u_2 = '$\omega$';
        u_3 = '$v_z$';
      end
      subplot(212);  hold all
      plot(agents{i}.u_store(1,2:end),'--','DisplayName', u_1,'color','b')
      ylabel('[m/s]')
      plot(agents{i}.u_bar_store(1,2:end),'-','DisplayName',[u_1,' real'],'color','b')
      if mdl == 2
        yyaxis right
        hold on
        set(gca, 'YColor', 'r')
        ylabel('[rad/s]')
        plot(agents{i}.u_store(2,2:end),'--','DisplayName', u_2,'color','r')
        plot(agents{i}.u_bar_store(2,2:end),'-','DisplayName',[u_2,' real'],'color','r')
        yyaxis left
      elseif mdl == 1
        plot(agents{i}.u_store(2,2:end),'--','DisplayName', u_2,'color','k')
        plot(agents{i}.u_bar_store(2,2:end),'-','DisplayName',[u_2,' real'],'color','k')
      end
      plot(agents{i}.u_store(3,2:end),'--','DisplayName', u_3,'color','g')
      plot(agents{i}.u_bar_store(3,2:end),'-','DisplayName',[u_3,' real'],'color','g')
      title('Inputs')
      xlabel('iteration')
      legend('Location', 'eastoutside')
      grid on
      box on
      sgtitle(['Agent ', num2str(i)])
    end

    %% Weighting Factor
    figure('Name','Weight function','NumberTitle','off','Color','w'); clf;
    plot(w_store)
    xlabel('step [s]')
    ylabel('w')
    title('Weighting factor for inverse kinematics')
    grid on
    box on

  elseif parametric == 1  %% Post processing for parametric study
    
    if length(post_process_data) > 1
      ind_plot = 1; % agents that has to be plotted
      % Error in the self localization
      fig_std_comparison = figure('Name', 'Parametric plot', 'NumberTitle', 'off', 'Color', 'w'); clf;
      axes('FontSize', par.font_size)
      hold on
      plot(NaN, NaN, '-', 'DisplayName', 'GPS', 'Color', color(1), 'MarkerSize', 20, 'LineWidth', line_width)
      plot(NaN, NaN, '-', 'DisplayName', 'rel meas', 'Color', color(2), 'MarkerSize', 20, 'LineWidth', line_width)
      plot(NaN, NaN, 'x', 'DisplayName', 'x', 'Color', 'k', 'MarkerSize', 20, 'LineWidth', line_width)
      plot(NaN, NaN, 'o', 'DisplayName', 'y', 'Color', 'k', 'MarkerSize', 20, 'LineWidth', line_width)
      plot(NaN, NaN, 's', 'DisplayName', 'z', 'Color', 'k', 'MarkerSize', 20, 'LineWidth', line_width)
      % change the GPS probability
      for k=1:par.prob_GPS_len
        plot(par.prob_GPS_vec(k), abs(post_process_data{k}.agents{ind_plot}.loc_error_std_after_wls{ind_plot}(1)), 'x', 'HandleVisibility', 'off', 'Color', color(1), 'MarkerSize', 20, 'LineWidth', line_width)
        plot(par.prob_GPS_vec(k), abs(post_process_data{k}.agents{ind_plot}.loc_error_std_after_wls{ind_plot}(2)), 'o', 'HandleVisibility', 'off', 'Color', color(1), 'MarkerSize', 20, 'LineWidth', line_width)
        plot(par.prob_GPS_vec(k), abs(post_process_data{k}.agents{ind_plot}.loc_error_std_after_wls{ind_plot}(3)), 's', 'HandleVisibility', 'off', 'Color', color(1), 'MarkerSize', 20, 'LineWidth', line_width)
      end
      % change the relative measurement probability
      for k=par.prob_GPS_len+par.prob_conn_len+1:par.prob_GPS_len+par.prob_conn_len+par.prob_rel_measurement_len
        plot(par.prob_rel_measurement_vec(k - par.prob_GPS_len - par.prob_conn_len), abs(post_process_data{k}.agents{ind_plot}.loc_error_std_after_wls{ind_plot}(1)), 'x', 'HandleVisibility', 'off', 'Color', color(2), 'MarkerSize', 20, 'LineWidth', line_width)
        plot(par.prob_rel_measurement_vec(k - par.prob_GPS_len - par.prob_conn_len), abs(post_process_data{k}.agents{ind_plot}.loc_error_std_after_wls{ind_plot}(2)), 'o', 'HandleVisibility', 'off', 'Color', color(2), 'MarkerSize', 20, 'LineWidth', line_width)
        plot(par.prob_rel_measurement_vec(k - par.prob_GPS_len - par.prob_conn_len), abs(post_process_data{k}.agents{ind_plot}.loc_error_std_after_wls{ind_plot}(3)), 's', 'HandleVisibility', 'off', 'Color', color(2), 'MarkerSize', 20, 'LineWidth', line_width)
      end
      title(['Std of self-localization error done by agent ', num2str(ind_plot)], 'FontSize', par.font_size)
      legend('Location', 'best', 'FontSize', par.font_size)
      box on
      grid on
      xlabel('Probability [-]', 'FontSize', par.font_size)
      ylabel('$\sigma$ [m]', 'FontSize', par.font_size)
      xlim([0 1.1]) 

      if enable_export == 1
        export_figure(fig_std_comparison, ['mdl', num2str(par.mdl), '_', num2str(par.n_agents), 'chutes_parametric_beforeconsensus.eps'], 'images\');
      end
    end
 

  % plot the error on the localizaations of the others
  fig_std_loc_others = figure('Name', 'Measure on the others', 'NumberTitle', 'off', 'Color', 'w'); clf;
      axes('FontSize', par.font_size)
      hold on
      plot(NaN, NaN, '-', 'DisplayName', 'GPS', 'Color', color(1), 'MarkerSize', 20, 'LineWidth', line_width)
      plot(NaN, NaN, '-', 'DisplayName', 'rel meas', 'Color', color(2), 'MarkerSize', 20, 'LineWidth', line_width)
      plot(NaN, NaN, 'x', 'DisplayName', 'x', 'Color', 'k', 'MarkerSize', 20, 'LineWidth', line_width)
      plot(NaN, NaN, 'o', 'DisplayName', 'y', 'Color', 'k', 'MarkerSize', 20, 'LineWidth', line_width)
      plot(NaN, NaN, 's', 'DisplayName', 'z', 'Color', 'k', 'MarkerSize', 20, 'LineWidth', line_width)
      % change the GPS probability
      for k=1:par.prob_GPS_len
        plot(par.prob_GPS_vec(k), abs(post_process_data{k}.agents{ind_plot}.std_loc_error_other_after_wls(1)), 'x', 'HandleVisibility', 'off', 'Color', color(1), 'MarkerSize', 20, 'LineWidth', line_width)
        plot(par.prob_GPS_vec(k), abs(post_process_data{k}.agents{ind_plot}.std_loc_error_other_after_wls(2)), 'o', 'HandleVisibility', 'off', 'Color', color(1), 'MarkerSize', 20, 'LineWidth', line_width)
        plot(par.prob_GPS_vec(k), abs(post_process_data{k}.agents{ind_plot}.std_loc_error_other_after_wls(3)), 's', 'HandleVisibility', 'off', 'Color', color(1), 'MarkerSize', 20, 'LineWidth', line_width)
      end
      % change the relative measurement probability
      for k=par.prob_GPS_len+par.prob_conn_len+1:par.prob_GPS_len+par.prob_conn_len+par.prob_rel_measurement_len
        plot(par.prob_rel_measurement_vec(k - par.prob_GPS_len - par.prob_conn_len), abs(post_process_data{k}.agents{ind_plot}.std_loc_error_other_after_wls(1)), 'x', 'HandleVisibility', 'off', 'Color', color(2), 'MarkerSize', 20, 'LineWidth', line_width)
        plot(par.prob_rel_measurement_vec(k - par.prob_GPS_len - par.prob_conn_len), abs(post_process_data{k}.agents{ind_plot}.std_loc_error_other_after_wls(2)), 'o', 'HandleVisibility', 'off', 'Color', color(2), 'MarkerSize', 20, 'LineWidth', line_width)
        plot(par.prob_rel_measurement_vec(k - par.prob_GPS_len - par.prob_conn_len), abs(post_process_data{k}.agents{ind_plot}.std_loc_error_other_after_wls(3)), 's', 'HandleVisibility', 'off', 'Color', color(2), 'MarkerSize', 20, 'LineWidth', line_width)
      end
      title(['Mean of std of localization error done by agent ', num2str(ind_plot), ' on all the others'], 'FontSize', par.font_size)
      legend('Location', 'best', 'FontSize', par.font_size)
      box on
      grid on
      xlabel('Probability [-]', 'FontSize', par.font_size)
      ylabel('$\sigma$ [m]', 'FontSize', par.font_size)
      xlim([0 1.1]) 

      if enable_export == 1
        export_figure(fig_std_loc_others, ['mdl', num2str(par.mdl), '_', num2str(par.n_agents), 'chutes_parametric_loc_others.eps'], 'images\');
      end

      % plot the error on the localizaations of the others before and after the wls
      k_ba_wls = 5; % parametric simulation that has to be plotted
      fig_ba_wls = figure('Name', 'b/a WLS', 'NumberTitle', 'off', 'Color', 'w'); clf;
      hold on;
      axes('FontSize', par.font_size)
      plot(NaN, NaN, '-', 'DisplayName', 'Before WLS', 'Color', color(1), 'MarkerSize', 20, 'LineWidth', line_width)
      plot(NaN, NaN, '-', 'DisplayName', 'After WLS', 'Color', color(2), 'MarkerSize', 20, 'LineWidth', line_width)
      plot(NaN, NaN, 'x', 'DisplayName', 'x', 'Color', 'k', 'MarkerSize', 20, 'LineWidth', line_width)
      plot(NaN, NaN, 'o', 'DisplayName', 'y', 'Color', 'k', 'MarkerSize', 20, 'LineWidth', line_width)
      plot(NaN, NaN, 's', 'DisplayName', 'z', 'Color', 'k', 'MarkerSize', 20, 'LineWidth', line_width)
      for j=1:n_agents
        plot(j, abs(post_process_data{k_ba_wls}.agents{ind_plot}.loc_error_std{j}(1)), 'x', 'HandleVisibility', 'off', 'Color', color(1), 'MarkerSize', 20, 'LineWidth', line_width);
        plot(j, abs(post_process_data{k_ba_wls}.agents{ind_plot}.loc_error_std{j}(2)), 'o', 'HandleVisibility', 'off', 'Color', color(1), 'MarkerSize', 20, 'LineWidth', line_width);
        plot(j, abs(post_process_data{k_ba_wls}.agents{ind_plot}.loc_error_std{j}(3)), 's', 'HandleVisibility', 'off', 'Color', color(1), 'MarkerSize', 20, 'LineWidth', line_width);
        plot(j, abs(post_process_data{k_ba_wls}.agents{ind_plot}.loc_error_std_after_wls{j}(1)), 'x', 'HandleVisibility', 'off', 'Color', color(2), 'MarkerSize', 20, 'LineWidth', line_width);
        plot(j, abs(post_process_data{k_ba_wls}.agents{ind_plot}.loc_error_std_after_wls{j}(2)), 'o', 'HandleVisibility', 'off', 'Color', color(2), 'MarkerSize', 20, 'LineWidth', line_width);
        plot(j, abs(post_process_data{k_ba_wls}.agents{ind_plot}.loc_error_std_after_wls{j}(3)), 's', 'HandleVisibility', 'off', 'Color', color(2), 'MarkerSize', 20, 'LineWidth', line_width);
      end
      title(['Std of localization error done by agent ', num2str(ind_plot), ' on another'], 'FontSize', par.font_size)
      legend('Location', 'best', 'FontSize', par.font_size)
      box on
      grid on
      xlabel('Localized agent', 'FontSize', par.font_size)
      ylabel('$\sigma$ [m]', 'FontSize', par.font_size)
      xlim([0 j+0.5]) 

      if enable_export == 1
        export_figure(fig_ba_wls, ['mdl', num2str(par.mdl), '_', num2str(par.n_agents), 'chutes_be_wls.eps'], 'images\');
      end
  end
end