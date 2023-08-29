function [agents, true_centroid_store] = global_centroid_chutes(agents, true_centroid_store, t, par)
  %% This function computes the global centroid and stores it in a variable only for plot porpuses

n_agents = par.n_agents;

  for i=1:n_agents
    %% Compute the global centroid
    true_centroid_store(:, t) = zeros(3, 1);
    for ii=1:n_agents
      true_centroid_store(:, t) = true_centroid_store(:, t) + agents{ii}.x_real(1:3)/n_agents;
    end
  end
end