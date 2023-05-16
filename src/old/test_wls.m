
clear 
close all
clc
A = [1 2 3 4 5 6;
   1 2 3 4 5 6;
   1 2 3 4 5 6];
agents =  cell(4, 1);
for i=1:4
 agents{i}.x = A; 
 for j=1:6
   agents{i}.P_est{j} = eye(3);
 end
end

agents = wls(agents);


function agents = wls(agents)

  n_agents = length(agents); % number of agents

  [n_states, n_mes] = size(agents{1}.x); % [number of states, number of measurements]
  % build the measurement matrix
  H = kron(ones(n_mes,1), eye(n_states));

  for i=1:n_agents
    C = []; % covariance matrix
    % rearrange the measurements in a vector
    Z = reshape(agents{i}.x(1:3, :), n_mes*n_states, 1);
    for j=1:n_mes
      % build the covariance matrix
      C = blkdiag(C, agents{i}.P_est{j}(1:3, 1:3));
    end

    % compute the WLS
    agents{i}.global_centroid = inv(H'*inv(C)*H)*H'*inv(C)*Z;
  end

end