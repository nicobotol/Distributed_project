% This function implements the distributed voronoi tesselletion
clearvars
close all
clc

colors_vect = [[0 0.4470 0.7410]; [0.8500 0.3250 0.0980]; ...
               [0.9290 0.6940 0.1250]; [0.4940 0.1840 0.5560]; ...
               [0.4660 0.6740 0.1880]; [0.3010 0.7450 0.9330]; ...
               [0.6350 0.0780 0.1840]];

Rs = 3; % sensing range
Rc = 5; % communication range
% Rc >= 2*Rs;

n_agents = 7;                   % number of agents
p = 5*(rand(2, n_agents) - 0.5);  % coordinates of the agents
fake_zero = 1e-6;       % fake 0        
fake_inf = 2e3;         % fake inf

%% Cetroid of the total population 
% This quantity is here supposed to be known in a centralized way
p_centroid = mean(p, 2);
r_formation = 5;            % radius in which all the agents have to be around the centroid
formation_circle = circle(p_centroid(1), p_centroid(2), r_formation);
lim_formation =  polyshape(formation_circle(:, 1), formation_circle(:,2));

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
lim_intersect = cell(7, 1);
lim_voronoi = cell(7, 1); 
lim_circle_rs = cell(7, 1);

for i=1:n_agents
  switch length(N{i})
    case 1  % don't set the voronoi limits and go with the sensing range
      circle_rs =  circle(p(1, i), p(2, i), Rs);
      lim_circle_rs{i} = polyshape(circle_rs(:, 1), circle_rs(:, 2));
      lim_voronoi{i} = lim_circle_rs{i};
      lim_intersect{i} = intersect(lim_formation, lim_circle_rs{i});
    case 2  % find the points where the circular area starts and stops
      if norm(p(:, N{i}(1))  - p(:, N{i}(2)))/2 < Rs
        [~, idx] = ismember(N{i}, i);
        idx = idx(1);
        if idx == 1 
          idy = 2;
        else
          idy =1;
        end
        circle_rs1 =  circle(p(1, N{i}(idx)), p(2, N{i}(idx)), Rs);
        lim_circle_rs1 = polyshape(circle_rs1(:, 1), circle_rs1(:, 2));
        circle_rs2 =  circle(p(1, N{i}(idy)), p(2, N{i}(idy)), Rs);
        lim_circle_rs2 = polyshape(circle_rs2(:, 1), circle_rs2(:, 2));
        lim_voronoi{i} = subtract(lim_circle_rs1, lim_circle_rs2);
        lim_intersect{i} = intersect(formation_circle, lim_voronoi{i});
      else
        circle_rs =  circle(p(1, i), p(2, i), Rs);
        lim_circle_rs{i} = polyshape(circle_rs(:, 1), circle_rs(:, 2));
        lim_intersect{i} = intersect(formation_circle, lim_circle_rs{i});
      end
    otherwise % set the proper middle points
      [V,C,XY] = VoronoiLimit(p(1, N{i}), p(2, N{i}), 'bs_ext', formation_circle, 'figure', 'off');
      [~, idx] = ismember(p(1, i), XY(:, 1)); % position to remap results of the tessellation in the order of the intial point
      lim_voronoi{i} = polyshape(V(C{idx}, 1), V(C{idx}, 2)); % pure voronoi cell
      circle_rs =  circle(p(1, i), p(2, i), Rs);
      lim_circle_rs{i} = polyshape(circle_rs(:, 1), circle_rs(:, 2));
      lim_intersect{i} = intersect(lim_voronoi{i}, lim_circle_rs{i});
  end
end

%% Find area and centroid of each cell
cell_centroid = zeros(2, n_agents);
cell_area = zeros(n_agents, 1);
for i=1:n_agents
  [ccx, ccy] = centroid(lim_intersect{i}); % centroid of the cell
  cell_centroid(:, i) = [ccx, ccy]';
  cell_area(i) = area(lim_intersect{i});          % area of the cell
end

%% Plots
j = 0; % figure numeber

j = j + 1;
figure(j); clf;
voronoi(p(1, :), p(2, :))
hold on
plot(p(1, :), p(2, :), 'o');
axis([min(p(1,:))-2, max(p(1,:))+2, min(p(2,:))-2, max(p(2,:))+2]);
title('Centralised Voronoi tessellation')
axis equal

j = j + 1;
figure(j); clf;
for i=1:n_agents
  hold on
  plot(lim_intersect{i}, 'DisplayName', ['Cell ', num2str(i)])
end
axis equal
plot(p(1, :), p(2, :), 'x', 'DisplayName', 'Agent')
plot(cell_centroid(1, :), cell_centroid(2, :), 'o', 'DisplayName', 'Centrodid')
plot(formation_circle(:,1), formation_circle(:, 2), 'k--', 'DisplayName', 'Formation')
legend('Location', 'best')


