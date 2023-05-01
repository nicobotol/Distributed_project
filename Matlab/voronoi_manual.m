clc
clear 
close all
range = 20;
Rc = 5;
Rs = Rc/2;
n_agents = 10;

agents = cell(n_agents,1);
% x = [0,1,0,-0.3,-1];
% y = [0,0.5,1,0.8,0.3];
for i = 1:n_agents
  x = (rand() - 0.5)*range;
  y = (rand() - 0.5)*range;
  agents{i}.x = [x, y]';      % positions of the agents 
  % agents{i}.x = [x(i), y(i)]';      % positions of the agents 
  agents{i}.neighbors = [];   % neighbors of the agents
  agents{i}.len_n = 0;        % number of neighbors
end

j_fig = 0;
% j_fig = j_fig+1;
% figure(j_fig); clf;
% for i = 2:n_agents
%   plot(agents{i}.x(1), agents{i}.x(2), 'or', 'MarkerSize', 10)
%   hold on
% end
% plot(agents{1}.x(1), agents{1}.x(2), 'xr', 'MarkerSize', 20)
% axis equal

% Check if the agents are within the communication range

for i = 1:n_agents
  for j = 1:n_agents
    if i ~= j
      dist = norm(agents{i}.x - agents{j}.x);
      if dist <= Rc
        agents{i}.neighbors = [agents{i}.neighbors, j];
        agents{i}.len_n = agents{i}.len_n + 1;          % number of agents in Rc
      end
    end
  end
end

tic
for i = 1:n_agents
  P = [];
  vx = [];
  vy = [];
  V = [];
  C = [];
  v = [];
  ia = [];
  inf_points = [];
  robots_pos = [];
  if agents{i}.len_n == 0     % no other agents in the range
    points = circle(agents{i}.x(1), agents{i}.x(2), Rs);
    agents{i}.voronoi = polyshape(points(:,1),points(:,2));
  elseif agents{i}.len_n == 1 % only one agent in the range
    dir = - agents{i}.x + agents{agents{i}.neighbors}.x;
    dir = dir/norm(dir);          % direction of the line
    norm_dir = [-dir(2); dir(1)];       % normal direction
    M =  mean([agents{i}.x'; agents{agents{i}.neighbors}.x'], 1)'; % middle point
    A = M + norm_dir*Rs;      % circle-middle line intersection sx
    B = M - norm_dir*Rs;      % circle-middle line intersection dx
    
    points = circle_sector(agents{i}.x(1), agents{i}.x(2), A, B);
    agents{i}.voronoi = polyshape(points(:,1),points(:,2));
  else                        % at least 2 agents in the range
    P(1,:) = agents{i}.x;
    for j = 1:agents{i}.len_n
      P(j+1,:) = agents{agents{i}.neighbors(j)}.x;  
    end
    [vx,vy] = voronoi(P(:,1), P(:,2));
    [V,C] = voronoin(P);

    % remove infinite points in V
    if isinf(V(1,1)) || isinf(V(1,2))
      V(1,:) = [];
      C{1}(find(C{1} == 1)) = [];
      C{1} = C{1} - 1;      
    end

    % find the elements of vx that are unique
    v = zeros(length(vx(1,:))*2,2);
    v = [vx(1,:)', vy(1,:)'; vx(2,:)', vy(2,:)'];
    % remove the duplicate points
    v = unique(v, 'rows'); 
    
    [~, ia] = setdiff(round(v, 6), round(V,6), 'rows');
    inf_points = v(ia,:);
    % associate the infinite points to vx and vy and elongate them
    for j = 1:length(inf_points(:,1))
      [r_inf,c_inf] = find(vx == inf_points(j,1));
      % if r_inf is not unique, check the y
      if length(r_inf) > 1
        [r_inf,c_inf] = find(vy == inf_points(j,2));
      end
      
      if r_inf == 1
        p_linked = [vx(2,c_inf), vy(2,c_inf)];
      else
        p_linked = [vx(1,c_inf), vy(1,c_inf)];
      end
      
      inf_points(j, :) = inf_points(j,:) + (inf_points(j,:) - p_linked)/norm(inf_points(j,:) - p_linked)*Rs*10;
      V = [V; inf_points(j,:)];
    end

    % Associate the new points to the agents
    row_start = length(V(:,1)) - length(inf_points(:, 1)) + 1;
    robots_pos(1,:) = agents{i}.x;
    for j = 1:agents{i}.len_n
      robots_pos(j+1,:) = agents{agents{i}.neighbors(j)}.x;
    end
    for j = row_start:length(V(:,1))
      V_dist = sum(abs(V(j,:)-robots_pos).^2,2).^0.5;
      V_index = find(V_dist == min(V_dist));
      if length(V_index)== 2
        if length(find(V_index == 1)) == 1
          C{1} = [C{1}, j];
        end
      else
        V_dist(V_index) = max(V_dist); 
        if length(find(V_index == 1)) == 1
          C{1} = [C{1}, j];
        end
        V_index = find(V_dist == min(V_dist));
        if length(find(V_index == 1)) == 1
          C{1} = [C{1}, j];
        end
      end
    end

    % create the polyshape of the cell
    k = convhull(V(C{1},:));
    poly_voronoi = polyshape(V(C{1}(k), 1), V(C{1}(k), 2));
    % create the polyshape of the sensing circle
    points = circle(agents{i}.x(1), agents{i}.x(2), Rs);
    poly_circle = polyshape(points(:,1),points(:,2));
    % find the intersection between the two polyshapes
    agents{i}.voronoi = intersect(poly_circle, poly_voronoi);

  end

end
toc
j_fig = j_fig+1;
figure(j_fig); clf;
for i=1:n_agents
  plot(agents{i}.voronoi);
  hold on
  plot(agents{i}.x(1), agents{i}.x(2), 'xr', 'MarkerSize', 20);
  pause
  % text([agents{i}.x(1), agents{i}.x(2)], [num2str(i)], 'FontSize', 20);
end
axis equal


