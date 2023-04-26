% This function implements the distributed voronoi tesselletion
clearvars
close all
clc

colors_vect = [[0 0.4470 0.7410]; [0.8500 0.3250 0.0980]; ...
               [0.9290 0.6940 0.1250]; [0.4940 0.1840 0.5560]; ...
               [0.4660 0.6740 0.1880]; [0.3010 0.7450 0.9330]; ...
               [0.6350 0.0780 0.1840]];

Rs = 1; % sensing range
Rc = 2.1; % communication range

p(1, :) = [1, 2, 3.5, 5, 5, 6, 6];       % x coordinates of the agents
p(2, :) = [1.0, 3, 2, 4, 0, -2, 1];      % y coordinates of the agents
n_agents = length(p); % number of agents
fake_zero = 1e-6;       % fake 0        
fake_inf = 2e3;         % fake inf

%% Cetroid of the total population 
% This quantity is here supposed to be known in a centralized way
p_centroid = mean(p, 2);
r_formation = 5;            % radius in which all the agents have to be around the centroid

%% Build the neighbours set
% The i-th neighbours set is made by all the agents inside the 
% communication range of the i-th agent
N = cell(n_agents, 1);  % neighbours set
for i = 1:n_agents % loop over agents
  k = 0;  % counter for the position
  for j = 1:n_agents 
    if norm(p(:, i) - p(:, j)) - Rc <= fake_zero
      k = k + 1;
      N{i}(k) = j; % store the id of the agent inside the comm. range
    end
  end
end
% N{i} contains the id of the agents inside the communication range of the
% agent i

%% Distribute the voronoi cell
v = cell(n_agents, 1);
c = cell(n_agents, 1);
v_voronoi = cell(n_agents, 1);
for i=1:n_agents
  switch length(N{i})
    case 1  % don't set the voronoi limits and go with the sensing range
      v_voronoi{i} = p(:, i);
    case 2  % set the voronoi limit on the middle point
      v_voronoi{i}(1, :) = inf;
      v_voronoi{i}(2, 1) = 0.5*(p(1, N{i}(1)) + p(2, N{i}(2)));
      v_voronoi{i}(2, 2) = 0.5*(p(1, N{i}(1)) + p(2, N{i}(2)));
    otherwise % set the proper middle points
    [v{i}, c{i}] = voronoin([p(1, N{i})', p(2, N{i})']);
    % each agent performs the tessellation for all the agents inside its own
    % communication range. The parameters of the tessellation are stored in
    % the cells v and c. To do so the i_th agent needs to know the position
    % of the adjecent agents.
    % v{i} is a vector of vertices of the cells of the tesellation of agent i
    % c{i} is the number of edges correspoing to the tassellation of the
    % agent i
  
    % Store in each agent its own edges
    [~, pos] = min(abs(N{i}(:) - i));
    v_voronoi{i} = v{i}(c{i}{pos}, :);
  end
  
end

%% Limit the voronoi cell by the sensing range and the external range
lim_area = cell(1, n_agents);       % voronoi cell with all the boundaries
p_agent_sr = cell(1, n_agents);     % sensing range area of the agent
cell_area = zeros(n_agents, 1);  % area of the pure tessellation
p_formation = circle(p_centroid(1), p_centroid(2), r_formation);        % boundary of the formation
lim_formation = polyshape(p_formation(:, 1), p_formation(:, 2));  % formation limit
for i=1:n_agents
  p_agent_sr{i} = circle(p(1, i), p(2, i), Rs);                       % sensing range of the agent 
  lim_agent_sr = polyshape(p_agent_sr{i});
  lim_voronoi = voronoi_area(p(:, i)', v_voronoi{i}, Rs, fake_inf);  % pure voronoi limit
  lim_area{i} = intersect(lim_formation, lim_voronoi);              % intersect the area
  lim_area{i} = intersect(lim_area{i}, lim_agent_sr);
cell_area(i) = area(lim_area{i});                                   % area of the cell considering all the boundaries
end

%% Plots
figure(1); clf;
plot(p(1, :), p(2, :), 'o');
title('Agents distribution on the plane');

figure(2)
voronoi(p(1, :), p(2, :))
axis([-2 8 -4 6])
title('Centralised Voronoi tessellation')

figure(3); clf;
for i=1:n_agents
  plot(p(1, i), p(2, i), 'x', 'DisplayName', num2str(i), ...
    'MarkerSize',7.5,'LineWidth',2, 'Color', colors_vect(i, :));
  % Each agent extracts from its own tessellation only the information
  % corresponding to itself
  hold on
  [~, pos] = min(abs(N{i}(:) - i));
  if length(N{i}) > 2
    plot(v{i}(c{i}{pos}, 1), v{i}(c{i}{pos},2 ), 'o', 'Color', ...
    colors_vect(i, :), 'MarkerSize', 10*(1+i/2), 'HandleVisibility','off');
  end
end
legend('location', 'northwest')
grid on
title('Agents distribution and voroinoi edges')

figure(4); clf;
rows = ceil(n_agents/2);
for i=1:n_agents
  subplot(rows, 2, i)
  plot(p(1, i), p(2, i), 'x', 'DisplayName', num2str(i), ...
    'MarkerSize',7.5,'LineWidth',2, 'Color', colors_vect(i, :));
  % Each agent extracts from its own tessellation only the information
  % corresponding to itself
  hold on
  if length(N{i}) > 1
    plot(v_voronoi{i}(:, 1), v_voronoi{i}(:, 2), 'o', 'Color', ...
      colors_vect(i, :), 'MarkerSize', 10);
  end
  grid on
end
legend('location', 'northwest')
grid on

figure(5); clf;
hold all
for i=1:n_agents
  plot(lim_area{i}, 'FaceColor', colors_vect(i, :), 'FaceAlpha',0.1);
  plot(p(1, i), p(2, i), 'x', 'Color', colors_vect(i, :), 'LineWidth', 2.5, 'MarkerSize',10);
  plot(p_agent_sr{i}(:,1), p_agent_sr{i}(:,2), '--' , 'Color', colors_vect(i, :));
end
plot(p_formation(:,1), p_formation(:,2), '--' , 'Color', 'k');
axis equal
title('Distributed voronoi tessellation limited')