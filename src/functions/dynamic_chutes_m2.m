function [agents, ground_check, true_centroid_store] = dynamic_chutes_m2(agents,ground_check, true_centroid_store, t)
% This function computes the new local centroid and the low level control of the agents

parameters;

for i=1:n_agents
  if agents{i}.x(3, i) > target(3) % Check if we have touch the ground
    % propagate the dynamic with the inputs
    agents{i}.x_real = A*agents{i}.x_real + B*u_unc + G*agents{i}.nu;
    agents{i}.x_store = [agents{i}.x_store, agents{i}.x_real]; % save the history of the agent's state
  else 
    ground_check(i) = 1; % mark that the agent has touched the ground
    agents{i}.x(3, i) = 0;  % set the z coordinate to 0
    agents{i}.sim_x = agents{i}.x;
  end
end


%% Compute the global centroid
true_centroid_store(:, t) = zeros(3, 1);
for ii=1:n_agents
  true_centroid_store(:, t) = true_centroid_store(:, t) + agents{ii}.x_real/n_agents;
end


end