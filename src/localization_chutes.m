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
        agents{i}.agents_x = [agents{i}.agents_x abs_mes]; % position of the agents j known by i

        % Propagate the uncertainty on the relative measurement
        agents{i}.agents_P_est{j} = agents{i}.P_est + agents{i}.eps_cov;
        
      end
    end
  end
end