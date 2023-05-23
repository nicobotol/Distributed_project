%% Share in each agent the information about the position of the others
function agents = distribute_informations(agents)
parameters;
n_agents = length(agents);


%% Distribute the position of the globalcentroid with the metropolis hastings algorithm
F = cell(n_agents,1);
a = cell(n_agents,1);

% Initialize the F and a matrices
for i=1:n_agents % consensus for robot i
  for j=1:n_agents
    Hj = eye(3);              % model
    Rj = agents{j}.P_est{i};  % covariance 
    F{j} = Hj'*inv(Rj)*Hj;
    a{j} = Hj'*inv(Rj)*agents{j}.x(1:3, i);
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
    for ii=1:n_agents
      for j=1:n_agents
        if A(j, ii) == 1
          F{ii} = F{ii} + 1/(1+max(D(ii), D(j)))*(FStore{j} - FStore{ii});
          a{ii} = a{ii} + 1/(1+max(D(ii), D(j)))*(aStore{j} - aStore{ii});
        end
      end
    end

  end % end of the exchange of messages
  
  % Estimation the position of agent i doing the consensus 
  for j = 1:n_agents
    agents{j}.x(1:3, i) = inv(F{j})*a{j};
    agents{j}.P_est{i} = inv(F{j});
  end
  
end % end consensus on robot i

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