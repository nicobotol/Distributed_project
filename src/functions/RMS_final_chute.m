function RMS = RMS_final_chute(agents, par, true_centroid_store)
  % This function computes the root mean square error of the agents' final positions and the distance between the centroid and the target point

  n_agents = par.n_agents;
  target = par.target;

  MSE = 0; % mean square error
  for i = 1:n_agents
    MSE = MSE + norm(target(1:2) - agents{i}.x_real(1:2))^2;
  end
  RMS = sqrt(MSE/n_agents); % root mean square errror


  dist = norm(target(1:2) - true_centroid_store(1:2, end));

  fprintf('RMS = %f [m]\n', RMS);
  fprintf('Distance between centroid and target = %f [m]\n', dist);

end
