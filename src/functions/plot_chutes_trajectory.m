function [] = plot_chutes_trajectory(agents,true_centroid_store, j_fig, w_store, par, post_process_data, parametric)

  marker_size = par.marker_size;
  dt = par.dt;
  v_lim = par.v_lim;
  Beta = par.Beta;
  vz_min = par.vz_min;
  target = par.target;
  mdl = par.mdl;
  enable_export = par.enable_export;
  n_agents = length(agents);
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
    xlim([-2*x0(1)-0.6*position_range, x0(1)+0.6*position_range])
    ylim([-x0(2)-0.6*position_range, x0(2)+0.6*position_range])
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

    %% Falling velocity
    figure('Name','Falling velocity','NumberTitle','off','Color','w'); clf;
    hold on
    for i=1:n_agents
      x_len = size(agents{i}.x_real_store, 2); % number of time step in the trjectory
      time = dt*(0:x_len-1); % time vector
      v_z =  diff(agents{i}.x_real_store(3,:))/dt; % computed falling velocity
      v_z = [0, v_z];
      plot(0:x_len, [v_z, 0], 'DisplayName', ['Agent ', num2str(i)]);
    end
    v_z_ff = falling_velocity(v_lim, Beta, dt, [0:1:x_len]); % free falling velocity
    plot(0:x_len, v_z_ff, 'DisplayName', 'Free fall')
    plot(0:x_len, -vz_min*ones(1, x_len+1), 'DisplayName', 'Min velocity')
    legend('location', 'bestoutside')
    xlabel('step [s]')
    ylabel('$V_z$ [m/s]')
    title('Falling velocity')
    grid on
    box on

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
      fig_std_comparison = figure('Name', 'Parametric plot', 'NumberTitle', 'off', 'Color', 'w'); clf;
      hold on
      plot(NaN, NaN, '-', 'DisplayName', 'GPS', 'Color', color(1), 'MarkerSize', 20, 'LineWidth', line_width)
      plot(NaN, NaN, '-', 'DisplayName', 'conn', 'Color', color(2), 'MarkerSize', 20, 'LineWidth', line_width)
      plot(NaN, NaN, '-', 'DisplayName', 'rel meas', 'Color', color(3), 'MarkerSize', 20, 'LineWidth', line_width)
      plot(NaN, NaN, 'x', 'DisplayName', 'x', 'Color', 'k', 'MarkerSize', 20, 'LineWidth', line_width)
      plot(NaN, NaN, 'o', 'DisplayName', 'y', 'Color', 'k', 'MarkerSize', 20, 'LineWidth', line_width)
      plot(NaN, NaN, 's', 'DisplayName', 'z', 'Color', 'k', 'MarkerSize', 20, 'LineWidth', line_width)
      % change the GPS probability
      for k=1:par.prob_GPS_len
        plot(par.prob_GPS_vec(k), post_process_data{k}.mean_std_x(1), 'x', 'HandleVisibility', 'off', 'Color', color(1), 'MarkerSize', 20, 'LineWidth', line_width)
        plot(par.prob_GPS_vec(k), post_process_data{k}.mean_std_x(2), 'o', 'HandleVisibility', 'off', 'Color', color(1), 'MarkerSize', 20, 'LineWidth', line_width)
        plot(par.prob_GPS_vec(k), post_process_data{k}.mean_std_x(3), 's', 'HandleVisibility', 'off', 'Color', color(1), 'MarkerSize', 20, 'LineWidth', line_width)
      end
      % change the connection prbability
      for k=par.prob_GPS_len+1:par.prob_GPS_len+par.prob_conn_len
        plot(par.prob_conn_vec(k - par.prob_GPS_len), post_process_data{k}.mean_std_x(1), 'x', 'HandleVisibility', 'off', 'Color', color(2), 'MarkerSize', 20, 'LineWidth', line_width)
        plot(par.prob_conn_vec(k - par.prob_GPS_len), post_process_data{k}.mean_std_x(2), 'o', 'HandleVisibility', 'off', 'Color', color(2), 'MarkerSize', 20, 'LineWidth', line_width)
        plot(par.prob_conn_vec(k - par.prob_GPS_len), post_process_data{k}.mean_std_x(3), 's', 'HandleVisibility', 'off', 'Color', color(2), 'MarkerSize', 20, 'LineWidth', line_width)
      end
      % change the relative measurement probability
      for k=par.prob_GPS_len+par.prob_conn_len+1:par.prob_GPS_len+par.prob_conn_len+par.prob_rel_measurement_len
        plot(par.prob_rel_measurement_vec(k - par.prob_GPS_len - par.prob_conn_len), post_process_data{k}.mean_std_x(1), 'x', 'HandleVisibility', 'off', 'Color', color(3), 'MarkerSize', 20, 'LineWidth', line_width)
        plot(par.prob_rel_measurement_vec(k - par.prob_GPS_len - par.prob_conn_len), post_process_data{k}.mean_std_x(2), 'o', 'HandleVisibility', 'off', 'Color', color(3), 'MarkerSize', 20, 'LineWidth', line_width)
        plot(par.prob_rel_measurement_vec(k - par.prob_GPS_len - par.prob_conn_len), post_process_data{k}.mean_std_x(3), 's', 'HandleVisibility', 'off', 'Color', color(3), 'MarkerSize', 20, 'LineWidth', line_width)
      end

      title('Mean of std of states')
      legend('Location', 'best')
      box on
      grid on
      xlabel('Probability [-]')
      ylabel('Mean of std of states [m]')
      xlim([0 1.1]) 

      if enable_export == 1
        export_figure(fig_std_comparison, 'fig_std_comparison.eps', 'images\');
      end
    end
  end

end