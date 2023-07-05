function agents = voronoi_centroid(agents,t)
%% This function computes the centroid of the voronoi cell

parameters;

for i=1:n_agents
  %% Compute the global centroid trajectory
  K = []; % LQR gain matrix
  if agents{i}.x(3, i) > target(3) % Check if we have touch the ground
    agents{i}.sim_x = []; % simulated trajectory of the chute
    
    % LQR gain matrix
    K = lqr(A, B, S, R, T, Sf, states_len, inputs_len, t);
    
    % LQR input: input that the i-th agent would apply at its own centroid, but in turns it applies to itself (i.e. the agents apply to itself the inputs that applies to the centroid)
    u_global_centroid = -K(:, :, t)*(agents{i}.global_centroid - target);   
    % Simulate the new chute position (i.e. the position where the chute has to go to fulfill the movement of the centroid as desired by the LQR)
    sim_x = A*agents{i}.x(:, i) + B*u_global_centroid;
    agents{i}.sim_x = [agents{i}.sim_x sim_x];
    
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
    for j=1:size(agents{i}.msh.Elements, 2)
      vertices = agents{i}.msh.Nodes(:, agents{i}.msh.Elements(:,j));
      element_centroid = mean(vertices, 2); % centroid of the j-th element of the mesh
  
      phi = mvnpdf(element_centroid, sim_x(1:2),Sigma); % evaluate on the centroid of the j-th element of the mesh a pdf centered in sim_x with standard deviation Sigma
  
      weight = weight + phi*mi(j); % mass of the voronoi cell
      
      agents{i}.centroid(1:2) = agents{i}.centroid(1:2) + phi*mi(j)*element_centroid; % weighted centroid of the voronoi cell (partial step)
    end
    agents{i}.centroid(:) = agents{i}.centroid/weight; % weighted centroid of the voronoi cell
  end
end