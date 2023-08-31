function RMS = RMS_final_chute(agents, par)
  % This function computes the root mean square error of the agents' final positions

  n_agents = par.n_agents;
  target = par.target;

  MSE = 0; % mean square error
  for i = 1:n_agents
    MSE = MSE + norm(target(1:2) - agents{i}.x_real(1:2))^2;
  end
  RMS = sqrt(MSE/n_agents); % root mean square errror

  fprintf('RMS = %f\n', RMS);

end
