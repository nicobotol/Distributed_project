function [agents, ground_check] = dynamic_chutes(agents, ground_check, t)
% This function computes the new local centroid and the low level control of the agents

parameters;

for i=1:n_agents
  if ground_check(i) == 1
    agents{i}.u_bar = zeros(inputs_len, 1);
    agents{i}.nu = zeros(nc_inputs_len, 1);
  end

  % propagate the dynamic with the inputs
  agents{i}.x_real = A*agents{i}.x_real + B*agents{i}.u_bar + G*agents{i}.nu;
  if agents{i}.x_real(3) <= 0 
    agents{i}.x_real(3) = 0;
    ground_check(i) = 1; % mark that the agent has touched the ground
  end
  agents{i}.x_real_store = [agents{i}.x_real_store, agents{i}.x_real];   % save the history of the agent's real state
end
end