function agents = distribute_positions_WLS(agents, par, t)
% This function distribute the positions of the agents usign the WLS and metropolis hastings weights.
t;
n_agents = par.n_agents;
measure_len = par.measure_len;
prob_connection = par.prob_connection;
m = par.m;
mdl = par.mdl;

% store the localization after the consensus
for i = 1:n_agents
  % agents{i}.x_store = [agents{i}.x_store, agents{i}.x(:,i)];
  for j = 1:n_agents
    if agents{i}.loc_error{j}(1, end) == 0 && j~=i % in case we don't updated the dynamic  
      agents{i}.loc_error{j}(3:5, end) = [NaN, NaN, NaN]'; 
    else
      agents{i}.loc_error{j}(3:5, end) = agents{i}.x(1:3, j) - agents{j}.x_real(1:3); 
    end  
  end
end


for i=1:n_agents
  for j=1:n_agents
    tmp_P_diag_other = [agents{i}.P_est{j}(1,1); agents{i}.P_est{j}(2,2); agents{i}.P_est{j}(3,3)];
    agents{i}.P_est_other{j} = [agents{i}.P_est_other{j}, tmp_P_diag_other]; % save the history of the agent's covariance
  end
end

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
    agents{i}.loc_error_after_wls{j} = [agents{i}.loc_error_after_wls{j}, loc_error_tmp'];
  end
  if mdl == 2
    agents{i}.P_est{i}(1:4, 4) = P_est_old{i}(1:4, 4); % re-insert the orientation
    agents{i}.P_est{i}(4, 1:3) = P_est_old{i}(4, 1:3);
  end

end

% store the localization after the consensus
for i = 1:n_agents
  % agents{i}.x_store = [agents{i}.x_store, agents{i}.x(:,i)];
  for j = 1:n_agents
    if agents{i}.loc_error_after_wls{j}(1, end) == 0 % in case we don't updated the dynamic  
      agents{i}.loc_error_after_wls{j}(3:5, end) = [NaN, NaN, NaN]'; 
    else
      agents{i}.loc_error_after_wls{j}(3:5, end) = agents{i}.x(1:3, j) - agents{j}.x_real(1:3); 
      % If the WLS has been done, then the agent knows the position of the chute and adds it to the visited chutes
      if ismember(j, agents{i}.visited_chutes) == 0
        agents{i}.visited_chutes = [agents{i}.visited_chutes, j];
        agents{i}.u_visit(:, j) = agents{j}.u;
      end 
    end  
  end

  tmp_P_diag = [agents{i}.P_est{i}(1,1); agents{i}.P_est{i}(2,2); agents{i}.P_est{i}(3,3)];
  agents{i}.P_est_store_after_wls = [agents{i}.P_est_store_after_wls, tmp_P_diag]; % save the history of the agent's covariance 

  for j=1:n_agents
    tmp_P_diag_other = [agents{i}.P_est{j}(1,1); agents{i}.P_est{j}(2,2); agents{i}.P_est{j}(3,3)];
    agents{i}.P_est_other_after_wls{j} = [agents{i}.P_est_other_after_wls{j}, tmp_P_diag_other]; % save the history of the agent's covariance
  end
end