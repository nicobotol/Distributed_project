function [agents, ground_check, true_centroid_store] = dynamic_chutes_m4(agents, t_step, ground_check, true_centroid_store, t)
% This function computes the new local centroid and the low level control of the agents

parameters;

for i=1:n_agents
  %% Compute the global centroid trajectory
  K = []; % LQR gain matrix
  if agents{i}.x(3, i) > target(3) % Check if we have touch the ground
    
    %% Low level control
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