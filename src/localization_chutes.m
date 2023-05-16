function agents = localization_chutes(agents)
% Localize the chutes and perform the relative measurements

n_agents = length(agents);

% Localize the chute
for i = 1:n_agents
  agents{i}.x = agents{i}.x_real; % in the simplest case the estimate positon is the real one
end

%% Simulate the communication between agents
for i = 1:n_agents
  for j = 1:n_agents
    if i ~= j
      dist3D = norm(agents{i}.x_real - agents{j}.x_real); % distance between robots in 3D space
      % add a robot in the neightbours set only if it is inside in the communication range and if it is at the seme height
      if dist3D <= agents{i}.Rc
        rel_mes = (agents{j}.x_real - agents{i}.x_real) + mvnrnd(zeros(inputs_len, 1), agents{i}.eps_cov)'; % perform the relative measure as the real distance between the agents plus a noise

        abs_mes = agents{i}.x + rel_mes; % Project the relative meas in abs.
        agents{i}.agents_x(:, j) = abs_mes; % position of the agents j known by i

        % Propagate the uncertainty on the relative measurement
        agents{i}.agents_P_est{j} = agents{i}.P_est + agents{i}.eps_cov;
      else % if one agent does not see another, then it assumes that the other agents is further than twice the communication range, and it also sets the covariance of the estimation to a high value 
        agents{i}.agents_x(:, j) = agents{i}.x + 2*agents{i}.Rc*ones(state_len, 1);
        agents{i}.agents_P_est{j} = 100*ones(state_len, state_len);    
      end
    end
  end
end