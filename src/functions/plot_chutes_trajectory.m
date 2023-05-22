function [] = plot_chutes_trajectory(agents,true_centroid_store)

parameters; % load the constant parameters

n_agents = length(agents);
j_fig = 2;

%% 3D trajectories
j_fig = j_fig+1;
figure(j_fig); clf;
hold all
for i=1:n
  plot3(agents{i}.x_store(1,:), agents{i}.x_store(2, :), agents{i}.x_store(3, :),'o','DisplayName', ['Agent ', num2str(i)])
end
% plot3(agents{i}.x_store(1, 1), agents{i}.x_store(1,2), agents{i}.x_store(1, 3), 'x', 'MarkerSize', marker_size, 'DisplayName', ['START', num2str(i)]);
plot3(target(1), target(2), target(3), 'o', 'MarkerSize', marker_size,'DisplayName', 'TARGET');
plot3(true_centroid_store(1, :), true_centroid_store(2,:), true_centroid_store(3, :), 'r--', 'DisplayName', 'Centroid')
plot3(agents{1}.sim_x(1),agents{1}.sim_x(1), 0, 'o')
xlabel('x [m] ')
ylabel('y [m]')
zlabel('z [m]')
legend('Location', 'best')
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
legend('Location', 'best')
grid on

end