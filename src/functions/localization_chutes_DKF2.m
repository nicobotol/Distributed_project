function agents = localization_chutes_DKF2(agents)
% Localize the chutes with a distributed kalman filtera and relative measurements

parameters; % load the parameters

n_agents = length(agents);

% Localize the chute TO BE UPDATED FOR THE MODEL WITH THETA
% agents{i}.x contains the state after the reaching of the consensus on the previous step, and so agents{i}.x(:, i) depends also on the KFs of the other N-1 agents, and so it cannot be used as estimation for a further step of the KF. For this reason the pure estimation of the position is stored in a separate filed and used for the update at the next step

for i = 1:n_agents % decide which agent has to be tracked
  x_pred = cell(n_agents,1);   % estimated state of the agent i
  P_mes = cell(n_agents,1);       % propagated covariance on the measurement
  P_pred = cell(n_agents,1);       % propagated covariance on the measurement
  zi = cell(n_agents,1);       % measurements of the agent i
  
  % Prediction
  for j=1:n_agents
    x_pred{j} = A*agents{j}.x(:, i) + B*agents{i}.u; % predict the state, assuming every agent has the same input
    P_pred{j} = A*agents{j}.P_est{i}*A' + B*agents{i}.Q*B'; % predict the covariance
  end
  % Update
  F = cell(n_agents,1);
  a = cell(n_agents,1);

  for j = 1:n_agents
    H = eye(measure_len, measure_len); % model of the relative measuremental
    if j ~= i
      dist3D = norm(agents{i}.x_real - agents{j}.x_real); % distance between robots in 3D space
      % add a robot in the neightbours set only if it is inside in the communication range and if it is at the seme height
      if dist3D <= agents{j}.Rc
        rel_mes = (agents{i}.x_real - agents{j}.x_real) + mvnrnd(zeros(states_len, 1), agents{j}.R_relative)'; % perform the relative measure as the real distance between the agents plus a noise

        abs_mes = agents{j}.x(1:3, j) + rel_mes; % Project the relative meas in abs.
        zi{j} = abs_mes; % position of the agents i known by j
        
        % Propagate the uncertainty on the relative measurement
        P_mes{j} = agents{j}.H*agents{j}.P_est{j}*agents{j}.H' + agents{j}.R_relative;
       
      else % if one agent does not see another, then it assumes that the other agents is further than twice the communication range, and it also sets the covariance of the estimation to a high value 
          
        zi{j} = zeros(states_len, 1); % position of the agents j known by i
        P_mes{j} = P_est_init*eye(states_len, states_len); 
      end
    else 
      zi{j} = agents{i}.x_real + mvnrnd(zeros(states_len, 1), agents{i}.R_GPS)'; % the agent knows its own position
      P_mes{j} = agents{i}.R_GPS; % the agent knows its own covariance
    end

  
    % uncertainty on the sensor readings Ri{i} + Hi{i}*P_est{i}*Hi{i}'
    F{j} = agents{j}.H'*inv(P_mes{j})*agents{j}.H;
    a{j} = agents{j}.H'*inv(P_mes{j})*zi{j};
  end

  for k = 1:m-1
    % Adjacency matrix
    C = zeros(n_agents);
    for ii = 1:n_agents
      for j = 1:n_agents
        if ii ~= j % the agent does not communicate with itself
          dist3D = norm(agents{ii}.x_real - agents{j}.x_real); % distance between robots in 3D space
          if dist3D <= agents{ii}.Rc
            C(ii, j) = 1;
          end
        end
      end
    end

    % Degree vector
    D = C*ones(n_agents, 1);

    % Compute F and a according the metropolis hastings algorithm
    FStore = F; % store of F before the update
    aStore = a; % store of a before the update
    for ii=1:n_agents
      for j=1:n_agents
        if C(j, ii) == 1
          F{ii} = F{ii} + 1/(1+max(D(ii), D(j)))*(FStore{j} - FStore{ii});
          a{ii} = a{ii} + 1/(1+max(D(ii), D(j)))*(aStore{j} - aStore{ii});
        end
      end
    end
  end
  
  %% Measurements update
  for j = 1:n_agents
    PredP = P_pred{j};
    agents{j}.P_est{i} = inv(P_pred{j} + n_agents*F{j});
    agents{j}.x(:,i) = agents{j}.P_est{i}*(PredP*x_pred{j} + n_agents*a{j});
  end
  
  
end