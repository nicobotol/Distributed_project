function agents = store_control_chutes(agents, ground_check)
  %% this function stores the estimaeted state

  parameters;

  for i=1:n_agents
    %% Estimated trajectory
    if agents{i}.x(3, i) > target(3) % Check if we have touch the ground
      agents{i}.u_store = [agents{i}.u_store, agents{i}.u];             % save the history of the agent's input
      agents{i}.u_bar_store = [agents{i}.u_bar_store, agents{i}.u_bar];             % save the history of the agent's input
    else 
      agents{i}.x(3, i) = 0;  % set the z coordinate to 0
      agents{i}.x_store = [agents{i}.x_store, agents{i}.x(:,i)];
      agents{i}.sim_x = agents{i}.x;
      agents{i}.u_bar = [0;0;0];
    end

    if ground_check(i) == 1
      agents{i}.x(3, i) = 0;
      agents{i}.x_store(3, end) = 0;
    end

  end
end