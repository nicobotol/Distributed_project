%% This function is used for showing the effectivness of the voronoi tessellation

clear; 
close all;
clc;

parameters;
n_agents = 2;
agents = cell(n_agents, 1);

% position of the agents
agents{1}.x_real(:) = [0 0 0]';
agents{2}.x_real(:) = [1 0 0]';
% agents{3}.x_real(:) = [0 2 0]';

% build a matrix with all the positions
for i=1:n_agents
  all_pos(:, i) = agents{i}.x_real(:);
end
% save the estimate in all the robots
for i=1:n_agents
  agents{i}.x = all_pos;
end

% save the encumbrance and sensing/communication range
agents{1}.Rc = 2.2;
agents{2}.Rc = 0.9;
% agents{3}.Rc = 1;
agents{1}.Rcv = 1;
agents{2}.Rcv = 1;
% agents{3}.Rcv = 1;
for i=1:n_agents
  agents{i}.Rs = agents{i}.Rc/2;
  agents{i}.Rsv = agents{i}.Rcv/2;
end

% save the max space covered at the maximum velocity 
for i=1:n_agents
  agents{i}.vmaxdt = 0;
end

% save the control input 
for i=1:n_agents
  agents{i}.u = zeros(3,1);
  agents{i}.u_visit = zeros(3);
end


%% Store the voronoi in the different conditions
% CASE 1: No encumbrance, no velocity, perfect position knowledge
  for i=1:n_agents
    agents{i}.delta = 0;
    agents{i}.vmaxdt = 0;
    for j=1:n_agents
      agents{i}.P_est{j} = zeros(3,3);
    end
  end
  %% Perform the voronoi tessellation
  agents = voronoi_chutes(agents);

  voronoi_case1 = cell(n_agents, 1);
  for i=1:n_agents
    voronoi_case1{i} = agents{i}.voronoi.Vertices;
  end
  
% CASE 2: Encumbrance and velocity, perfect position knowledge
  agents{1}.delta = 0.25;
  agents{2}.delta = 0.15;
  for i=1:n_agents
    agents{i}.vmaxdt = 0.35;
  end
  %% Perform the voronoi tessellation
  agents = voronoi_chutes(agents);
  
  voronoi_case2 = cell(n_agents, 1);
  for i=1:n_agents
    voronoi_case2{i} = agents{i}.voronoi.Vertices;
  end

% CASE 3: Encumbrance and velocity, imperfect position knowledge
  for i=1:n_agents
    agents{i}.P_est{i} = 0.05^2*eye(3,3);
  end
  agents{1}.P_est{2} = 0.1^2*eye(3,3);
  agents{2}.P_est{1} = 0*eye(3,3);
  %% Perform the voronoi tessellation
  agents = voronoi_chutes(agents);

  voronoi_case3 = cell(n_agents, 1);
  for i=1:n_agents
    voronoi_case3{i} = agents{i}.voronoi.Vertices;
  end


figure(); hold on;
plot([NaN, NaN],'x', 'MarkerSize', 10, 'LineWidth', 2, 'Color', 'k');
plot([NaN, NaN],':', 'Color', 'k', 'LineWidth',2);
plot([NaN, NaN],'--', 'Color', 'k', 'LineWidth',2);
plot([NaN, NaN],'-', 'Color', 'k', 'LineWidth',2);
plot([NaN, NaN],'-', 'Color', 'k', 'LineWidth', 1);
plot([NaN, NaN],'--', 'Color', 'k', 'LineWidth', 1);
patch([NaN, NaN], [NaN, NaN], 'k', 'FaceAlpha', 0.1, 'EdgeColor', 'k');
for i=1:n_agents
  
  % position of the agents
  plot(agents{i}.x_real(1), agents{i}.x_real(2), 'x', 'MarkerSize', 10, 'LineWidth', 2, 'Color', colors_vect(i, :));
  
  % voronoi cell
  plot(voronoi_case1{i}(:, 1), voronoi_case1{i}(:, 2), ':', 'Color', colors_vect(i, :), 'LineWidth',2);
  plot(voronoi_case2{i}(:, 1),voronoi_case2{i}(:, 2), '--', 'Color', colors_vect(i, :), 'LineWidth',2);
  plot(voronoi_case3{i}(:, 1), voronoi_case3{i}(:, 2), '-','Color', colors_vect(i, :), 'LineWidth',2);
  
  % sensing and communication range
  Rs_circle = circle(agents{i}.x_real(1), agents{i}.x_real(2), agents{i}.Rs);
  Rc_circle = circle(agents{i}.x_real(1), agents{i}.x_real(2), agents{i}.Rc);
  plot(Rs_circle(:, 1), Rs_circle(:, 2), '-','Color', colors_vect(i, :), 'LineWidth', 1);
  plot(Rc_circle(:, 1), Rc_circle(:, 2), '--','Color', colors_vect(i, :), 'LineWidth', 1);

  % encumbrance
  encumbrance = circle(agents{i}.x_real(1), agents{i}.x_real(2), agents{i}.delta);
  % plot(encumbrance(:, 1), encumbrance(:, 2), '-.', 'Color', colors_vect(i, :));
  patch(encumbrance(:, 1), encumbrance(:, 2), colors_vect(i, :), 'FaceAlpha', 0.1, 'EdgeColor', 'k');

end
legend('pos.', 'voronoi 1', 'voronoi 2', 'voronoi 3', 'Rs', 'Rc', 'delta', 'Location', 'eastoutside');
axis equal;
grid on;
xlabel('x [m]');
ylabel('y [m]');