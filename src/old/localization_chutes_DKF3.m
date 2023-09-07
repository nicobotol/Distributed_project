function agents = localization_chutes_DKF3(agents)
% Localize the chutes with a distributed kalman filtera and relative measurements

parameters; % load the parameters

n_agents = length(agents);

% Localize the chute TO BE UPDATED FOR THE MODEL WITH THETA
% agents{i}.x contains the state after the reaching of the consensus on the previous step, and so agents{i}.x(:, i) depends also on the KFs of the other N-1 agents, and so it cannot be used as estimation for a further step of the KF. For this reason the pure estimation of the position is stored in a separate filed and used for the update at the next step

% Initialization
% Suppose to know th inputs of all the agents
u_DKF = zeros(inputs_len*n_agents, 1); % input for the DKF
for i=1:n_agents
  u_DKF((i-1)*inputs_len+1:i*inputs_len) = agents{i}.u; % build the input for the DKF
end

P = cell(1,n_agents); % covariance matrix for the DKF
x_pred = cell(1,n_agents); % state for the DKF
Q_DKF = []; % Q matrix for the DKF
P_mes = cell(1,n_agents); % covariance matrix for the DKF
for i=1:n_agents
%   P{i} = agents{i}.P_DKF;
  zi{i} = zeros(n_agents*states_len, 1);
  P{i} = kron(ones(n_agents), zeros(states_len));
  for j=1:n_agents
    P{i}((j-1)*states_len+1:j*states_len,(j-1)*states_len+1:j*states_len) = agents{i}.P_est{j}(1:3, 1:3); % build the covariance matrix for the DKF
%     P{i}((j-1)*states_len+1:j*states_len,(i-1)*states_len+1:i*states_len) = agents{i}.P_est{j}(1:3, 1:3); % build the covariance matrix for the DKF
  end
end
A_DKF = kron(eye(n_agents), A); % build the A matrix for the DKF
B_DKF = kron(eye(n_agents), B); % build the B matrix for the DKF

for i=1:n_agents
  Q_DKF = blkdiag(Q_DKF, agents{i}.Q); % build the Q matrix for the DKF
end

% Prediction
for i=1:n_agents
  x_pred{i} = reshape(agents{i}.x, n_agents*states_len, 1);
  x_pred{i} = A_DKF*x_pred{i} + B_DKF*u_DKF; % predict the state

  P_pred{i} = A_DKF*P{i}*A_DKF' + B_DKF*Q_DKF*B_DKF'; % predict the covariance
end

% Update
F = cell(n_agents,1);
a = cell(n_agents,1);
for i = 1:n_agents
  P_mes{i} = 1000*eye(states_len*n_agents);
  for j = 1:n_agents
    if i ~= j
      dist3D = norm(agents{i}.x_real - agents{j}.x_real); % distance between robots in 3D space
      % add a robot in the neightbours set only if it is inside in the communication range and if it is at the seme height
      if dist3D <= agents{j}.Rc
        rel_mes = (agents{i}.x_real - agents{j}.x_real) + mvnrnd(zeros(states_len, 1), agents{j}.R_relative)'; % perform the relative measure as the real distance between the agents plus a noise

        abs_mes = agents{j}.x(1:3, j) + rel_mes; % Project the relative meas in abs.
        zi{i}((j-1)*states_len+1:j*states_len) = abs_mes; % position of the agents i known by j
        
        % Propagate the uncertainty on the relative measurement
        H = eye(3);
        P_mes{i}((j-1)*states_len+1:j*states_len,(j-1)*states_len+1:j*states_len) = H*agents{i}.P_est{i}*H' + agents{i}.R_relative;
      else
        zi{i}((j-1)*states_len+1:j*states_len) = zeros(states_len, 1); % position of the agents i known by j
        
        % Propagate the uncertainty on the relative measurement
        P_mes{i}((j-1)*states_len+1:j*states_len,(j-1)*states_len+1:j*states_len) = P_est_init*eye(states_len);
      end
    else % perform the GPS measurement
      zi{i}((j-1)*states_len+1:j*states_len) = agents{i}.x_real + mvnrnd(zeros(states_len, 1), agents{i}.R_GPS)'; % position of the agents j known by i
      P_mes{i}((i-1)*states_len+1:i*states_len,(i-1)*states_len+1:i*states_len) = agents{i}.R_GPS;
    end
  end 
  % uncertainty on the sensor readings Ri{i} + Hi{i}*P_est{i}*Hi{i}'
  F{i} = agents{i}.H'*inv(P_mes{i})*agents{i}.H;
  a{i} = agents{i}.H'*inv(P_mes{i})*zi{i};
end

for k = 1:m-1
  % Adjacency matrix
  C = zeros(n_agents);
  for ii = 1:n_agents
    for j = 1:n_agents
      if ii ~= j
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

% Measurements update
for i = 1:n_agents
  PredP = P_pred{i};
  P{i} = inv(P_pred{i} + n_agents*F{i});
  x_pred{i} = P{i}*(PredP*x_pred{i} + n_agents*a{i});
end

% for i = 1:n_agents
%   PredP = P_pred{i};
%   P{i} = inv(F{i});
%   x_pred{i} = P{i}*a{i};
% end

%% Extract the estimates
for i = 1:n_agents
  for j = 1:n_agents
    agents{i}.x(1:3, j) = x_pred{i}((j-1)*states_len+1:j*states_len);
    agents{i}.P_est{j}(1:3, 1:3) = P{i}((j-1)*states_len+1:j*states_len,(j-1)*states_len+1:j*states_len);
    % agents{i}.P_DKF = P{i};
  end
end


