%% This function is used for showing the effectivness of the voronoi tessellation

clear; 
close all;
clc;

% Set LaTeX as default interpreter for axis labels, ticks and legends
set(0,'defaulttextinterpreter','latex')
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');
set(0,'DefaultFigureWindowStyle','docked');
set(0,'defaultAxesFontSize',25)
set(0,'DefaultLegendFontSize',25)

% Set the value of the variable to be used in the simulation
variable_param.prob_GPS = 1;                % probability of getting GPS signal
variable_param.prob_connection = 1;               % probability of comunicating during the consensus
variable_param.prob_rel_measurement = 1;    % probability of measuring the relative position of the other chutes
par = parameters(variable_param);
par.n_agents = 2;
n_agents = par.n_agents;
enable_export = par.enable_export;
t = 1;

agents = cell(n_agents, 1);

% position of the agents
agents{1}.x_real(:) = [0 0 0]';
agents{2}.x_real(:) = [1 0 1]';
% agents{3}.x_real(:) = [0 2 0]';

% visited chutes
agents{1}.visited_chutes = 2;
agents{2}.visited_chutes = 1;

% heigh
agents{1}.z_th = 1;
agents{2}.z_th = 1;

% max space
agents{1}.vmaxzdt = 2; 
agents{2}.vmaxzdt = 2; 

% build a matrix with all the positions
for i=1:n_agents
  all_pos(:, i) = agents{i}.x_real(:);
end
% save the estimate in all the robots
for i=1:n_agents
  agents{i}.x = all_pos;
end
agents{2}.x(3, 1) = 100;

% save the encumbrance and sensing/communication range
agents{1}.Rc = 2.2;
agents{2}.Rc = 0.55;
agents{1}.Rcv = 1;
agents{2}.Rcv = 1;
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
  [agents, delta_final_case1] = voronoi_chutes(agents, t, par);

  voronoi_case1 = cell(n_agents, 1);
  for i=1:n_agents
    voronoi_case1{i} = agents{i}.voronoi.Vertices;
    voronoi_case1{i}(end + 1, :) = voronoi_case1{i}(1, :);
  end
  
% CASE 2: Encumbrance and velocity, perfect position knowledge
  agents{1}.delta = 0.25;
  agents{2}.delta = 0.15;
  agents{1}.vmaxdt = 1;
  agents{2}.vmaxdt = 100;
  %% Perform the voronoi tessellation
  [agents, delta_final_case2] = voronoi_chutes(agents, t, par);
  
  voronoi_case2 = cell(n_agents, 1);
  for i=1:n_agents
    voronoi_case2{i} = agents{i}.voronoi.Vertices;
    voronoi_case2{i}(end + 1, :) = voronoi_case2{i}(1, :);
  end

% CASE 3: Encumbrance and velocity, imperfect position knowledge
  for i=1:n_agents
    agents{i}.P_est{i} = 0.05^2*eye(3,3);
  end
  agents{1}.P_est{2} = 0.1^2*eye(3,3);
  agents{2}.P_est{1} = 0*eye(3,3);
  %% Perform the voronoi tessellation
  [agents, delta_final_case3] = voronoi_chutes(agents, t, par);

  voronoi_case3 = cell(n_agents, 1);
  for i=1:n_agents
    voronoi_case3{i} = agents{i}.voronoi.Vertices;
    voronoi_case3{i}(end + 1, :) = voronoi_case3{i}(1, :);
  end

%% Plot
fig_voronoi_example = figure('Color', 'w'); hold on;plot([NaN, NaN],'x', 'MarkerSize', 10, 'LineWidth', 2, 'Color',par.colors_vect(1, :));
plot([NaN, NaN],'x','MarkerSize',10,'LineWidth',2,'Color',par.colors_vect(2, :));
plot([NaN, NaN],':', 'Color', 'k', 'LineWidth',2);
plot([NaN, NaN],'--', 'Color', 'k', 'LineWidth',2);
plot([NaN, NaN],'-', 'Color', 'k', 'LineWidth',2);
plot([NaN, NaN],'-', 'Color', 'k', 'LineWidth', 1);
plot([NaN, NaN],'--', 'Color', 'k', 'LineWidth', 1);
patch([NaN, NaN], [NaN, NaN], 'k', 'FaceAlpha', 0.4, 'EdgeColor', 'k');
patch([NaN, NaN], [NaN, NaN], 'k', 'FaceAlpha', 0.1, 'EdgeColor', 'k');

for i=1:n_agents
  
  % position of the agents
  plot(agents{i}.x_real(1), agents{i}.x_real(2), 'x', 'MarkerSize', 10, 'LineWidth', 2, 'Color', par.colors_vect(i, :));
  
  % voronoi cell
  plot(voronoi_case1{i}(:, 1), voronoi_case1{i}(:, 2), ':', 'Color', par.colors_vect(i, :), 'LineWidth',2);
  plot(voronoi_case2{i}(:, 1),voronoi_case2{i}(:, 2), '--', 'Color', par.colors_vect(i, :), 'LineWidth',2);
  plot(voronoi_case3{i}(:, 1), voronoi_case3{i}(:, 2), '-','Color', par.colors_vect(i, :), 'LineWidth',2);
  
  % sensing and communication range
  Rs_circle = circle(agents{i}.x_real(1), agents{i}.x_real(2), agents{i}.Rs);
  Rc_circle = circle(agents{i}.x_real(1), agents{i}.x_real(2), agents{i}.Rc);
  plot(Rs_circle(:, 1), Rs_circle(:, 2), '-','Color', par.colors_vect(i, :), 'LineWidth', 1);
  plot(Rc_circle(:, 1), Rc_circle(:, 2), '--','Color', par.colors_vect(i, :), 'LineWidth', 1);

  % encumbrance
  encumbrance2 = circle(agents{i}.x_real(1), agents{i}.x_real(2), delta_final_case2(i));
  encumbrance3 = circle(agents{i}.x_real(1), agents{i}.x_real(2), delta_final_case3(i));
  encumbrance = circle(agents{i}.x_real(1), agents{i}.x_real(2), agents{i}.delta);

  patch(encumbrance2(:, 1), encumbrance2(:, 2), par.colors_vect(i, :), 'FaceAlpha', 0.4, 'EdgeColor', 'k');
  patch(encumbrance3(:, 1), encumbrance3(:, 2), par.colors_vect(i, :), 'FaceAlpha', 0.1, 'EdgeColor', 'k');
%   patch(encumbrance(:, 1), encumbrance(:, 2), par.colors_vect(i, :), 'FaceAlpha', 1, 'EdgeColor', 'k'); % physical dimension
  
end
legend('Agent 1', 'Agent 2', '$\mathcal{V}_A$', '$\mathcal{V}_B$', '$\mathcal{V}_C$', 'Rs', 'Rc', '$\delta$', '$\tilde{\delta}$','Location', 'eastoutside');
axis equal;
grid on; box on;
xlabel('x [m]');
ylabel('y [m]');
if enable_export == 1
  export_figure(fig_voronoi_example, '\fig_voronoi_example.eps', 'images\');
end
ylabel('y [m]');