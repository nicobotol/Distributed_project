function [agents, ground_check] = dynamic_chutes(agents, ground_check, t)
% This function computes the new local centroid and the low level control of the agents

parameters;

for i=1:n_agents
  %% Estimated trajectory
  if agents{i}.x(3, i) > target(3) % Check if we have touch the ground
    agents{i}.u_store = [agents{i}.u_store, agents{i}.u];             % save the history of the agent's input
     
  else 
    ground_check(i) = 1; % mark that the agent has touched the ground
    agents{i}.x(3, i) = 0;  % set the z coordinate to 0
    agents{i}.x_store = [agents{i}.x_store, agents{i}.x(:,i)];
    agents{i}.sim_x = agents{i}.x;
    agents{i}.u_unc = [0;0;0];
  end

  %% Real trajectory
  if agents{i}.x_real(3) > target(3) % Check if we have touch the ground

    % propagate the dynamic with the inputs
    agents{i}.x_real = A*agents{i}.x_real + B*agents{i}.u_unc + G*agents{i}.nu;
    agents{i}.x_real_store = [agents{i}.x_real_store, agents{i}.x_real];   % save the history of the agent's real state
     
  else 
    agents{i}.x_real = A*agents{i}.x_real;
    agents{i}.x_real(3) = 0;
    agents{i}.x_real_store = [agents{i}.x_real_store, agents{i}.x_real];   % save the history of the agent's real state
  end 


end
end