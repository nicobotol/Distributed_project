%% Share in each agent the information about the position of the others
function agents = distribute_informations(agents)
parameters;
n_agents = length(agents);

%% Simulate the communication between agents
for i = 1:n_agents
  for j = 1:n_agents
    if i ~= j
      dist3D = norm(agents{i}.x - agents{j}.x); % distance between robots in 3D space
      % add a robot in the neightbours set only if it is inside in the communication range and if it is at the seme height
      if dist3D <= agents{i}.Rc
        rel_mes = (agents{j}.x - agents{i}.x) + mvnrnd(zeros(inputs_len, 1), agents{i}.eps_cov)';

        abs_mes = agents{i}.x + rel_mes; % WORK HERE
        
        agents{i}.agents_x = [agents{i}.agents_x abs_mes]; % position of the agents j known by i
        
      end
    end
  end
end

%% Compute the global centroid (i.e. centroid of the storm)
for i=1:n_agents
  agents{i}.global_centroid = mean([agents{i}.x agents{i}.agents_position],2);
end

%% Distribute the position of the globalcentroid with the metropolis hastings algorithm
F = cell(n_agents,1);
a = cell(n_agents,1);

% Initialize the F and a matrices
for i=1:n_agents
  Hi = eye(3); % model
  Ri = eye(3); % covariance THIS MUST BE CHANGED
  F{i} = Hi'*inv(Ri)*Ri;
  a{i} = Hi'*inv(Ri)*agents{i}.global_centroid;
end

% Exchange the messages
for k = 1:m-1
  % Adjacency matrix
  A = zeros(n_agents);
  for i = 1:n_agents
    for j = 1:n_agents
      if i ~= j
        dist3D = norm(agents{i}.x - agents{j}.x); % distance between robots in 3D space
        if dist3D <= agents{i}.Rc
          A(i, j) = 1;
        end
      end
    end
  end

  % Degree vector
  D = A*ones(n_agents, 1);

  % Compute F and a according the metropolis hastings algorithm
  FStore = F; % store of F before the update
  aStore = a; % store of a before the update
  for i=1:n
    for j=1:n
        if A(j, i) == 1
            F{i} = F{i} + 1/(1+max(D(i), D(j)))*(FStore{j} - FStore{i});
            a{i} = a{i} + 1/(1+max(D(i), D(j)))*(aStore{j} - aStore{i});
        end
    end
  end

  % Estimation of the global centroid
  for i = 1:n_agents
    agents{i}.global_centroid = inv(F{i})*a{i};
  end

end

end