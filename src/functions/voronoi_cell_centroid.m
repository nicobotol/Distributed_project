function agents = voronoi_cell_centroid(agents,t)
%% This function computes the centroid of the voronoi cell

parameters;

for i=1:n_agents
  %% Compute the global centroid trajectory
  if agents{i}.x(3, i) > target(3) % Check if we have touch the ground
    
    %% Computation of the new weighted centroid of the Voronoi cells based on the new chute position
    tr = triangulation(agents{i}.voronoi);
    model = createpde;
    tnodes = tr.Points';
    telements = tr.ConnectivityList';
    geometryFromMesh(model,tnodes,telements);
    agents{i}.msh = generateMesh(model,"Hmin",1,"GeometricOrder","linear"); % generate the mesh
    [~, mi] = area(agents{i}.msh); % (row vector with) area of each element of the mesh 
    weight = 0; % weigth of an area 
    agents{i}.centroid(1:2) = zeros(2, 1); 
    % figure()
%     hold on
    for j=1:size(agents{i}.msh.Elements, 2)
      % tic
      vertices = agents{i}.msh.Nodes(:, agents{i}.msh.Elements(:,j));
      % toc

      % tic
      element_centroid = mean(vertices, 2); % centroid of the j-th element of the mesh
      % toc
      
      % tic      
%       tmp = eval(agents{i}.sim_x(1:2));
%       agents{i}.sim_x(1:2) = tmp;
%       phi = mvnpdf(element_centroid, eval(agents{i}.sim_x(1:2)), Sigma); % evaluate on the centroid of the j-th element of the mesh a pdf centered in sim_x with standard deviation Sigma
      phi = mvnpdf(element_centroid, agents{i}.sim_x(1:2), Sigma);
      % toc 
      
      % tic
      weight = weight + phi*mi(j); % mass of the voronoi cell
      % toc

      % tic
      agents{i}.centroid(1:2) = agents{i}.centroid(1:2) + phi*mi(j)*element_centroid; % weighted centroid of the voronoi cell (partial step)
      % toc
    end
    % plot(agents{i}.msh.Nodes(1, :), agents{i}.msh.Nodes(2, :), 'o')
%     axis equal
    agents{i}.centroid(:) = agents{i}.centroid/weight; % weighted centroid of the voronoi cell
  end
end