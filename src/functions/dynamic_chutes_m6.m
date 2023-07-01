function [agents, ground_check, true_centroid_store] = dynamic_chutes_m6(agents, t_step, ground_check, true_centroid_store, t)
% This function computes the new local centroid and the low level control of the agents

parameters;

for i=1:n_agents
  %% Compute the global centroid trajectory
  K = []; % LQR gain matrix
  if agents{i}.x(3, i) > target(3) % Check if we have touch the ground
    agents{i}.sim_x = []; % simulated trajectory of the chute
    
    % LQR gain matrix
    K = lqr(A, B, S, R, T, Sf, states_len, inputs_len, t);
    
    % LQR input: input that the i-th agent would apply at its own centroid, but in turns it applies to itself (i.e. the agents apply to itself the inputs that applies to the centroid)
    u_global_centroid = -K(:, :, t_step)*(agents{i}.global_centroid - target);  
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

    % external disturbance
    agents{i}.nu(:) = nu_mag*mvnrnd([0;0;0;0], agents{i}.L)';   
    agents{i}.nu(4) = -v_lim*(1-exp(-Beta*t)); % falling velocity of a mass in a fluid
  
    %% Low level control
    agents{i}.u(1:2) = agents{i}.kp*(agents{i}.centroid(1:inputs_len-1) - agents{i}.x(1:inputs_len-1, i)); % low level control

    % % Dynamic + control (G matrix = 0)
    % if agents{i}.x(3,i) < ground_th % if the agent is near the ground, we apply the minimum constant input on the z coordinate of the centroid
    %   agents{i}.u(3) = agents{i}.v_min;
    % else
    %   % compute the control input and constrain it between the upper and lower bounds
    %   u_cmp = -2*(agents{i}.x(3,i) - agents{i}.z_min); % negative value
    %   if agents{i}.nu(4) > agents{i}.v_min
    %     agents{i}.u(3) = agents{i}.nu(4);
    %   else
    %     agents{i}.u(3) = min(max(u_cmp, agents{i}.nu(4)), agents{i}.v_min);
    %   end
    % end

    % Control w/o dynamic
    if agents{i}.x(3,i) < ground_th % if the agent is near the ground, we apply the minimum constant input on the z coordinate of the centroid
      agents{i}.u(3) = agents{i}.v_min - agents{i}.nu(4);
    else
      % compute the control input and constrain it between the upper and lower bounds
      u_cmp = kp_z/(agents{i}.x(3,i) - agents{i}.z_min) - kp_z/agents{i}.Rsv; % positive value
      v_tmp = min(agents{i}.v_min, agents{i}.nu(4));
      agents{i}.u(3) = min(u_cmp, agents{i}.v_min - v_tmp); % set the control velocity
    end
  
    %% Update the state of the agent
    % input with external disturbance
    u_unc = agents{i}.u + mvnrnd(zeros(inputs_len, 1)', agents{i}.Q)';

    % propagate the dynamic with the inputs
    agents{i}.x_real = A*agents{i}.x_real + B*u_unc + G*agents{i}.nu;
    agents{i}.x_real_store = [agents{i}.x_real_store, agents{i}.x_real];   % save the history of the agent's real state
    agents{i}.x_store = [agents{i}.x_store, agents{i}.x(:, i)];       % save the history of the agent's state
    agents{i}.u_store = [agents{i}.u_store, agents{i}.u];             % save the history of the agent's input
     
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