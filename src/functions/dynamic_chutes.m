function [agents, ground_check] = dynamic_chutes(agents, ground_check, t)
% This function computes the new local centroid and the low level control of the agents

parameters;

for i=1:n_agents
  %% Compute the global centroid trajectory

  if agents{i}.x(3, i) > target(3) % Check if we have touch the ground

    % propagate the dynamic with the inputs
    agents{i}.x_real = A*agents{i}.x_real + B*agents{i}.u_unc + G*agents{i}.nu;
    agents{i}.x_real_store = [agents{i}.x_real_store, agents{i}.x_real];   % save the history of the agent's real state
    agents{i}.u_store = [agents{i}.u_store, agents{i}.u];             % save the history of the agent's input
     
  else 
    agents{i}.x_real = agents{i}.x_real_store(:,end);
    agents{i}.x_real(3) = 0;
    agents{i}.x_store = [agents{i}.x_store, [agents{i}.x(1);agents{i}.x(2); 0]];
    agents{i}.x_real_store = [agents{i}.x_real_store, agents{i}.x_real];   % save the history of the agent's real state
    ground_check(i) = 1; % mark that the agent has touched the ground
    agents{i}.x(3, i) = 0;  % set the z coordinate to 0
    agents{i}.sim_x = agents{i}.x;
  end
end
end