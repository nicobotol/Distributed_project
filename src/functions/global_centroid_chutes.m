function [agents, true_centroid_store] = global_centroid_chutes(agents, true_centroid_store, t)
  %% This function computes the global centroid 

  parameters;
  for i=1:n_agents
    %% Compute the global centroid
    true_centroid_store(:, t) = zeros(3, 1);
    for ii=1:n_agents
      true_centroid_store(:, t) = true_centroid_store(:, t) + agents{ii}.x_real/n_agents;
    end
  end
end