function [] = plot_chutes_trajectory(agents,true_centroid_store, j_fig, w_store)

parameters; % load the constant parameters

n_agents = length(agents);

%% 3D trajectories
j_fig = j_fig+1;
figure(j_fig); clf;
hold all
for i=1:n
  plot3(agents{i}.x_real_store(1,:), agents{i}.x_real_store(2, :), agents{i}.x_real_store(3, :),'DisplayName', ['Agent ', num2str(i)])
end
% plot3(agents{i}.x_store(1, 1), agents{i}.x_store(1,2), agents{i}.x_store(1, 3), 'x', 'MarkerSize', marker_size, 'DisplayName', ['START', num2str(i)]);
plot3(target(1), target(2), target(3), 'o', 'MarkerSize', marker_size,'DisplayName', 'TARGET');
plot3(true_centroid_store(1, :), true_centroid_store(2,:), true_centroid_store(3, :), 'r--', 'DisplayName', 'Centroid', 'LineWidth', 2)
% plot3(agents{1}.sim_x(1),agents{1}.sim_x(1), 0, 'o')
xlabel('x [m] ')
ylabel('y [m]')
zlabel('z [m]')
legend('Location', 'bestoutside')
grid on

%% Vertical displacement
j_fig = j_fig+1;
figure(j_fig); clf;
hold all
for i=1:n
  plot(agents{i}.x_store(2,:), agents{i}.x_store(3, :),'o','DisplayName', ['Agent ', num2str(i)])
end
% plot3(agents{i}.x_store(1, 1), agents{i}.x_store(1,2), agents{i}.x_store(1, 3), 'x', 'MarkerSize', marker_size, 'DisplayName', ['START', num2str(i)]);
plot(target(2), target(3), 'o', 'MarkerSize', marker_size,'DisplayName', 'TARGET');
plot(true_centroid_store(2,:), true_centroid_store(3, :), 'r--', 'DisplayName', 'Centroid')
xlabel('y [m]')
ylabel('z [m]')
legend('Location', 'bestoutside')
grid on

%% Inputs  and trajectories
for i=1:n
  j_fig = j_fig+1;
  figure(j_fig); clf;
  subplot(121);  hold all
  plot(agents{i}.x_store(1,1:end),'--','DisplayName', 'x', 'color', 'b')
  plot(agents{i}.x_store(2,1:end),'--','DisplayName', 'y', 'color', 'r')
  plot(agents{i}.x_store(3,1:end),'--','DisplayName', 'z', 'color', 'g')
  if mdl == 5
    plot(agents{i}.x_store(4,1:end),'--','DisplayName', '$\theta$', 'color', 'k')
  end
  plot(agents{i}.x_real_store(1,1:end),'DisplayName','x real','color','b')
  plot(agents{i}.x_real_store(2,1:end),'DisplayName','y real','color','r')
  plot(agents{i}.x_real_store(3,1:end),'DisplayName','z real','color','g')
  if mdl == 5
    plot(agents{i}.x_real_store(4,1:end),'DisplayName','$\theta$ real','color','k')
  end
  title('State')
  xlabel('iteration')
  ylabel('[m]')
  legend('Location', 'eastoutside')
  grid on

  switch mdl
  case 5 % unicycle dynamic
    u_1 = '$V$';
    u_2 = '$\omega$';
    u_3 = '$v_z$';
  case 6 % linear dynamic
    u_1 = '$v_x$';
    u_2 = '$v_y$';
    u_3 = '$v_z$';
  end
  subplot(122);  hold all
  plot(agents{i}.u_store(1,2:end),'--','DisplayName', u_1,'color','b')
  plot(agents{i}.u_store(2,2:end),'--','DisplayName', u_2,'color','r')
  plot(agents{i}.u_store(3,2:end),'--','DisplayName', u_3,'color','g')
  plot(agents{i}.u_bar_store(1,2:end),'DisplayName',[u_1,' real'],'color','b')
  plot(agents{i}.u_bar_store(2,2:end),'DisplayName',[u_2,' real'],'color','r')
  plot(agents{i}.u_bar_store(3,2:end),'DisplayName',[u_3,' real'],'color','g')
  title('Input')
  xlabel('iteration')
  ylabel('[m/s]')
  legend('Location', 'eastoutside')
  grid on

  sgtitle(['Agent ', num2str(i)])
end

%% Falling velocity
figure(); clf;
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
figure(); clf;
plot(w_store)
xlabel('step [s]')
ylabel('w')
title('Weighting factor for inverse kinematics')
grid on
box on

end