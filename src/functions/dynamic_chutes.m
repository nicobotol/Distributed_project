function [agents] = dynamic_chutes(agents, t_step)
% This function computes the new local centroid and the low level control of the agents

parameters;

tic
for i=1:n_agents
  %% Compute the global centroid trajectory

  % LQR gain matrix
  K = lqr(A, B, S, R, T, Sf, states_len);

  if agents{i}.x(3, i) > target(3) % Check if we have touch the ground
    
    % LQR input: input that the i-th agent would apply at its own centroid, but in turns it applies to itself (i.e. the agents apply to itself the inputs that applies to the centroid)
    u_global_centroid = -K(:, :, t_step)*(agents{i}.global_centroid - target);   
  else 
    continue;
  end


  % Simulate the new chute position (i.e. the position where the chute has to go to fulfill the movement of the centroid as desired by the LQR)
  sim_x = A*agents{i}.x(:, i) + B*u_global_centroid;
  
  %% Computation of the new weighted centroid of the Voronoi cells based on the new chute position
  tr = triangulation(agents{i}.voronoi);
  model = createpde;
  tnodes = tr.Points';
  telements = tr.ConnectivityList';
  geometryFromMesh(model,tnodes,telements);
  agents{i}.msh = generateMesh(model,"Hmin",1,"GeometricOrder","linear"); % generate the mesh
  [~, mi] = area(agents{i}.msh); % (row vector with) area of each element of the mesh 
  weight = 0; % weigth of an area 
  for j=1:size(agents{i}.msh.Elements, 2)
    vertices = agents{i}.msh.Nodes(:, agents{i}.msh.Elements(:,j));
    element_centroid = mean(vertices, 2); % centroid of the j-th element of the mesh

    phi = mvnpdf(element_centroid, sim_x(1:2),Sigma); % evaluate on the centroid of the j-th element of the mesh a pdf centered in sim_x with standard deviation Sigma

    weight = weight + phi*mi(j); % mass of the voronoi cell
    
    agents{i}.centroid(:) = agents{i}.centroid(:) + phi*mi(j)*element_centroid; % weighted centroid of the voronoi cell (partial step)
  end
  agents{i}.centroid(:) = agents{i}.centroid/weight; % weighted centroid of the voronoi cell
  agents{i}.centroid(3) = agents{i}.x(3, i) - 1/2*9.81*dt^2;

  %% Low level control
  % REMARK: Add here a control on theta
  agents{i}.u = agents{i}.kp*(agents{i}.centroid(:) - agents{i}.x(:, i)); % low level control

  %% Update the state of the agent
  agents{i}.x_real = A*agents{i}.x_real + B*agents{i}.u; 
  agents{i}.x_store = [agents{i}.x_store, agents{i}.x_real]; % save the history of the agent's state

  
end 
toc

%% Compute the global centroid
for i=1:n_agents
  true_centroid_store(:, t_step) = true_centroid_store(:, t_step) + agents{i}.x_real/n_agents;
end

end