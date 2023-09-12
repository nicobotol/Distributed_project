function agents = voronoi_cell_centroid(agents, t, par)
%% This function computes the centroid of the voronoi cell

  n_agents = par.n_agents;
  target = par.target;
  Sigma = par.Sigma;

  for i=1:n_agents
    %% Compute the global centroid trajectory
    if agents{i}.x(3, i) > target(3) % Check if we have touch the ground
      
      %% Computation of the new weighted centroid of the Voronoi cells based on the new chute position
      tr = triangulation(agents{i}.voronoi);
      model = createpde;
      tnodes = tr.Points';
      telements = tr.ConnectivityList';
      geometryFromMesh(model,tnodes,telements);
%       agents{i}.msh = generateMesh(model,"Hmin",1,"GeometricOrder","linear"); % generate the mesh
      agents{i}.msh = generateMesh(model,"GeometricOrder","linear");
      [~, mi] = area(agents{i}.msh); % (row vector with) area of each element of the mesh 
      weight = 0; % weigth of an area 
      agents{i}.centroid(1:2) = zeros(2, 1); 

      for j=1:size(agents{i}.msh.Elements, 2)
        vertices = agents{i}.msh.Nodes(:, agents{i}.msh.Elements(:,j));

        element_centroid = mean(vertices, 2); % centroid of the j-th element of the mesh

        phi = mvnpdf(element_centroid, agents{i}.sim_x(1:2), Sigma);

        weight = weight + phi*mi(j); % mass of the voronoi cell
        
        % weighted centroid of the voronoi cell (partial step)
        agents{i}.centroid(1:2) = agents{i}.centroid(1:2) + phi*mi(j)*element_centroid; 
      end
      agents{i}.centroid(:) = agents{i}.centroid/weight; % weighted centroid of the voronoi cell
    end
  end

end