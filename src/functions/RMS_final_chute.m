function RMS = RMS_final_chute(agents)
  parameters;

  MSE = 0; % mean square error
  for i = 1:n_agents
    MSE = MSE + norm(agents{i}.global_centroid(1:2) - agents{i}.x_real(1:2))^2;
  end
  RMS = sqrt(MSE/n_agents); % root mean square errror

  fprintf('RMS = %f\n', RMS);
