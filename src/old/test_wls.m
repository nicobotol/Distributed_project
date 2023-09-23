clear; clc;
agents{1}.x_real = [ 1.1 0.7];
agents{2}.x_real = [ 5.4 9.2];
agents{1}.x = [ 1.1 0.7;  5.4 9.2]';
agents{2}.x = [  0.9 0.8;  5.5 9.1]';
agents{1}.Rc = 50;
agents{2}.Rc = 50;
agents{1}.P_est{1} = [0.078 0.009; 0.009 0.027];
agents{1}.P_est{2} = [0.039 -0.04; -0.04 0.041];
agents{2}.P_est{1} = [0.078 0.009; 0.009 0.027] - 1e-3;
agents{2}.P_est{2} = [0.039 -0.04; -0.04 0.041] - 1e-3;
mdl = 1;
n_agents = 2;
m = 1000;
measure_len = 2;
prob_connection = 1;
t = 0;

% Distribute the position of the globalcentroid with the metropolis hastings algorithm
F = cell(n_agents,1);
a = cell(n_agents,1);
C = cell(n_agents,1);
P_est_old = cell(n_agents,1);

% Initialize the F and a matrices
for i = 1:n_agents % consensus for robot i

  zi = reshape(agents{i}.x(1:2, :), measure_len*n_agents, 1); % vector of the positions of all the agents
  for j = 1:n_agents
    % build the covariance matrix
    C{i} = blkdiag(C{i}, agents{i}.P_est{j}(1:2, 1:2)); % P_est already includes the covariance of the measurements
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
sum_A = sum(A,'all');
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
        % F{i} = F{i} + 1/(1+max(D))*(FStore{j} - FStore{i});
        % a{i} = a{i} + 1/(1+max(D))*(aStore{j} - aStore{i});
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
    
    % initialize the vector for storing the variables after the wls
    flag = does_communicate(A, i, j);
    loc_error_tmp = [flag, t, NaN, NaN, NaN];
  end
  if mdl == 2
    agents{i}.P_est{i}(1:4, 4) = P_est_old{i}(1:4, 4); % re-insert the orientation
    agents{i}.P_est{i}(4, 1:2) = P_est_old{i}(4, 1:2);
  end

end