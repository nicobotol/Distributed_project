function agents = wls_global_centroid(agents, par)
% This function computes the local estimation of the global centroid, so where each agent thinks the network centroid is

n_agents = par.n_agents;

% build the measurement matrix
H = kron(ones(n_agents,1), eye(3));

for i=1:n_agents
  Z = []; % measurements
  C = []; % covariance matrix
  for j=1:n_agents
    % rearrange the measurements in a vector
    Z = [Z; agents{i}.x(1:3, j)];
    
    % build the covariance matrix
    C = blkdiag(C, agents{i}.P_est{j}(1:3, 1:3));
  end
  % compute the WLS
  agents{i}.global_centroid = inv(H'*inv(C)*H)*H'*inv(C)*Z;
end
 

end