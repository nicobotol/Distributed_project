function [agents, ground_check, true_centroid_store] = dynamic_chutes_m4(agents, t_step, ground_check, true_centroid_store)
% This function computes the new local centroid and the low level control of the agents

parameters;

for i=1:n_agents
  %% Compute the global centroid trajectory
  K = []; % LQR gain matrix
  if agents{i}.x(3, i) > target(3) % Check if we have touch the ground
    agents{i}.sim_x = []; % simulated trajectory of the chute
    
    % LQR gain matrix
    K = lqr(A, B, S, R, T, Sf, states_len, inputs_len);

     % Angle of the chute wrt the x axis
    if agents{i}.x(1,i) - target(1) > 0 && agents{i}.x(2,i) - target(2) > 0
      alpha = atan2(agents{i}.x(2,i) - target(2), agents{i}.x(1,i) - target(1));          
    elseif agents{i}.x(1,i) - target(1) > 0 && agents{i}.x(2,i) - target(2) < 0
      alpha = atan2(agents{i}.x(2,i) - target(2), agents{i}.x(1,i) - target(1)) + 2*pi;  
    elseif agents{i}.x(1,i) - target(1) < 0 && agents{i}.x(2,i) - target(2) > 0
      alpha = atan2(agents{i}.x(2,i) - target(2), agents{i}.x(1,i) - target(1));      
    elseif agents{i}.x(1,i) - target(1) < 0 && agents{i}.x(2,i) - target(2) < 0
      alpha = atan2(agents{i}.x(2,i) - target(2), agents{i}.x(1,i) - target(1)) + 2*pi;
    end

    % Desired angle of the chute wrt the x axis
    theta_des = alpha + pi/2;
    agents{i}.x(4, i) = wrapTo2Pi(agents{i}.x(4, i));
    agents{i}.x_real(4, i) = wrapTo2Pi(agents{i}.x_real(4, i));
    
    % LQR input: input that the i-th agent would apply at its own centroid, but in turns it applies to itself (i.e. the agents apply to itself the inputs that applies to the centroid)
    u_global_centroid = -K(:, :, t_step)*(agents{i}.global_centroid(4) - theta_des);   
    % Simulate the new chute position (i.e. the position where the chute has to go to fulfill the movement of the centroid as desired by the LQR)
    G_est = G(agents{i}.x(4, i));
    G_real = G(agents{i}.x_real(4));

    sim_x = A*agents{i}.x(:, i) + B*u_global_centroid +G_est*nu;
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
  
    %% Low level control
    % REMARK: Add here a control on theta
    agents{i}.u = agents{i}.kp*(agents{i}.centroid(1:inputs_len) - agents{i}.x(1:inputs_len, i)); % low level control
  
    %% Update the state of the agent
    agents{i}.x_real = A*agents{i}.x_real + B*agents{i}.u_unc + G_real*agents{i}.nu_unc; 
    agents{i}.x_store = [agents{i}.x_store, agents{i}.x_real]; % save the history of the agent's state
     
  else 
    agents{i}.u = zeros(inputs_len, 1);
    ground_check(i) = 1; % mark that the agent has touched the ground
    agents{i}.x(3, i) = 0;  % set the z coordinate to 0
    agents{i}.sim_x = agents{i}.x;
  end
end
%% Compute the global centroid
true_centroid_store(:, t_step) = zeros(3, 1);
for ii=1:n_agents
  true_centroid_store(:, t_step) = true_centroid_store(:, t_step) + agents{ii}.x_real/n_agents;
end
end