function agents = distribute_positions_WLS(agents, par)
% This function distribute the positions of the agents usign the WLS and metropolis hastings weights.

n_agents = par.n_agents;
measure_len = par.measure_len;
prob_connection = par.prob_connection;
m = par.m;
mdl = par.mdl;


%% Distribute the position of the globalcentroid with the metropolis hastings algorithm
F = cell(n_agents,1);
a = cell(n_agents,1);
C = cell(n_agents,1);
P_est_old = cell(n_agents,1);

% Initialize the F and a matrices
for i = 1:n_agents % consensus for robot i
  if mdl == 2 % since in case of mdl 2 the orintation does not pass through the WLS, we have to store it separately and then re-insert it
    P_est_old{i} = agents{i}.P_est{i};
  end

  zi = reshape(agents{i}.x(1:3, :), measure_len*n_agents, 1); % vector of the positions of all the agents
  for j = 1:n_agents
    % build the covariance matrix
    C{i} = blkdiag(C{i}, agents{i}.P_est{j}(1:3, 1:3)); % P_est already includes the covariance of the measurements
  end
  Hi{i} = eye(n_agents*measure_len);
  
  % check for singularity of the marix C{i}
  F{i} = Hi{i}'*inv(C{i})*Hi{i};
  a{i} = Hi{i}'*inv(C{i})*zi;
end

% Exchange the messages
% Adjacency matrix
A = zeros(n_agents);
for ii = 1:n_agents
  for j = ii+1:n_agents
    dist3D = norm(agents{ii}.x_real - agents{j}.x_real); % distance between robots in 3D space
    if dist3D <= agents{ii}.Rc 
      if rand(1) <= prob_connection
        A(ii, j) = 1;
      else
        A(ii, j) = 0;
      end
    end
  end
end
A = A + A'; % build the symmetric adjacency matrix

% Degree vector
D = A*ones(n_agents, 1);

for k = 1:m-1
  % Compute F and a according the metropolis hastings algorithm
  FStore = F; % store of F before the update
  aStore = a; % store of a before the update
  for i=1:n_agents
    for j=1:n_agents
      if A(j, i) == 1
        F{i} = F{i} + 1/(1+max(D(i), D(j)))*(FStore{j} - FStore{i});
        a{i} = a{i} + 1/(1+max(D(i), D(j)))*(aStore{j} - aStore{i});
      end
    end
  end

end % end of the exchange of messages
  
% Estimation the position of agent i doing the consensus 
for i = 1:n_agents
  tmp = inv(F{i})*a{i};
  agents{i}.x(1:measure_len, :) = reshape(tmp, measure_len, n_agents);
  for j=1:n_agents
    pos = measure_len*(j-1)+1:measure_len*j;
    F_minus = inv(F{i});
    agents{i}.P_est{j} = F_minus(pos,pos);
  end
  if mdl == 2
    agents{i}.P_est{i}(1:4, 4) = P_est_old{i}(1:4, 4); % re-insert the orientation
    agents{i}.P_est{i}(4, 1:3) = P_est_old{i}(4, 1:3);
  end

  % store the localization after the consensus
  % agents{i}.x_store = [agents{i}.x_store, agents{i}.x(:,i)];

end
end