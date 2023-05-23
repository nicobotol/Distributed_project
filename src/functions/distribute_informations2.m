%% Share in each agent the information about the position of the others
% This is another implementation of the distribute_informations code
function agents = distribute_informations2(agents)
parameters;
n_agents = length(agents);


%% Distribute the position of the globalcentroid with the metropolis hastings algorithm
F = cell(n_agents,1);
a = cell(n_agents,1);
C = cell(n_agents,1);

% Initialize the F and a matrices
for i=1:n_agents % consensus for robot i
  zi = reshape(agents{i}.x(1:3, :), states_len*n_agents, 1); % vector of the positions of all the agents
  for j = 1:n_agents
    % build the covariance matrix
    C{i} = blkdiag(C{i}, agents{i}.P_est{j}(1:3, 1:3));
  end
  Hi{i} = eye(n_agents*states_len);
  F{i} = Hi{i}'*inv(C{i})*Hi{i};
  a{i} = Hi{i}'*inv(C{i})*zi;
end

% Exchange the messages
for k = 1:m-1
  % Adjacency matrix
  A = zeros(n_agents);
  for ii = 1:n_agents
    for j = 1:n_agents
      if ii ~= j
        dist3D = norm(agents{ii}.x_real - agents{j}.x_real); % distance between robots in 3D space
        if dist3D <= agents{ii}.Rc
          A(ii, j) = 1;
        end
      end
    end
  end

  % Degree vector
  D = A*ones(n_agents, 1);

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
  agents{i}.x = reshape(tmp, states_len, n_agents);
  for j=1:n_agents
    pos = states_len*(j-1)+1:states_len*j;
    F_minus = inv(F{i});
    agents{i}.P_est{j} = F_minus(pos,pos);
  end
end
  

%% Compute the global centroid (i.e. centroid of the storm) without the agents that are too far
% for i=1:n_agents
%   tmp_vec = [];
%   for j=1:n_agents
%     if norm(agents{i}.P_est{j}) < P_est_threshold
%       tmp_vec = [tmp_vec agents{i}.x(1:3, j)];
%     end
%   end
%   agents{i}.global_centroid = mean(tmp_vec, 2);
% end

% % ALTERNATIVE CENTROID
% for i=1:n_agents
%   agents{i}.global_centroid = mean(agents{i}.x(1:3,:), 2);
% end

%% Compute the global centroid
agents = wls_centroid(agents);

end