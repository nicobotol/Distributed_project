function [] = plot_chutes_time_evo(agents)

parameters; % load the constant parameters

n_agents = length(agents);
j_fig = 0;


figure(j_fig); clf;
hold all
for i=1:n
  plot3(agents{i}.x(1,:), agents{i}.x(2, :), agents{i}.x(3, :), 'DisplayName', ['Agent ', num2str(i)])
  plot3(agents{i}.x(1, 1), agents{i}.x(1,2), agents{i}.x(1, 3), 'x', 'MarkerSize', marker_size, 'DisplayName', ['START', num2str(i)]);
end
plot3(target(1), target(2), target(3), 'o', 'MarkerSize', marker_size,'DisplayName', 'TARGET');
plot3(true_centroid_store(1, :), true_centroid_store(2,:), true_centroid_store(3, :), 'r--', 'DisplayName', 'Centroid')
xlabel('x [m] ')
ylabel('y [m]')
zlabel('z [m]')
legend('Location', 'best')
grid on

end