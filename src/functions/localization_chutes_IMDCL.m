function [agents] = localization_chutes_IMDCL()
  % implement the Interim Master Decentralized Cooperative Localization algorithm for the chute localization

% initialization
for i=1:n_agents
  agents{i}.x_hat = zeros(states_len, 1);
  agents{i}.P_p = zeros(states_len, states_len);
  agents{i}.Pij = cell(n_agents, n_agents);
  for j = 1:length(j_agents)
    for l = 1:length(l_agents)
      agents{i}.Pi{j,l} = zeros(states_len, states_len);
    end
  end
  agents{i}.Phi = eye(states_len);
end

% Propagation
for i=1:n_agents
  agents{i}.x_hat_m = A*agents{i}.x_hat_p + B*agents{i}.u + G(3, 4)*nu(4);
  agents{i}.P_m = A*agents{i}.P*A' + B*agents{i}.Q*B';
  aganets{i}.Phi = A*agents{i}.Phi;
end

% Update
for i=1:n_agents
  for j = 1:n_agents
    if i ~= j
      dist = norm(agents{i}.x_real - agents{j}.x_real); % distance between agents

      if dist >= agents{i}.Rc % no relative measurements
        agents{i}.x_hat_p = agents{i}.x_hat_m;
        agents{i}.P_p = agents{i}.P_m;
        agents{i}.Pi_jl = agents{i}.Pi_jl;
      else  % relative measurements

      end

    end 
  end
end
