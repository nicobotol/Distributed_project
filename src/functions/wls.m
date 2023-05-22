function agents = wls(agents);

  n_agents = length(agents); % number of agents

  [n_states, n_mes] = size(agents{1}.x); % [number of states, number of measurements]
  % build the measurement matrix
  H = kron(ones(n_mes,1), eye(n_states));

  for i=1:n_agents
    Z = []; % measurements
    C = []; % covariance matrix
    for j=1:n_agents
      % rearrange the measurements in a vector
%       z = reshape(agents{i}.x(1:3, j), n_mes*n_states, 1);
      Z = [Z; agents{i}.x(1:3, j)];
      
      % build the covariance matrix
      C = blkdiag(C, agents{i}.P_est{j}(1:3, 1:3));
    end
    % compute the WLS
    agents{i}.global_centroid = inv(H'*inv(C)*H)*H'*inv(C)*Z;
  end
 

end