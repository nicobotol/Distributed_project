function agents = localization_chutes_DKF(agents)
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
x_est = cell(1,n_agents); % state for the DKF
Q_DKF = []; % Q matrix for the DKF
for i=1:n_agents
  P{i} = kron(ones(n_agents), P_est_init*eye(states_len));
  for j=1:n_agents
    P{i}((i-1)*states_len+1:i*states_len,(j-1)*states_len+1:j*states_len) = agents{i}.P_est{j}(1:3, 1:3); % build the covariance matrix for the DKF
  end
end
A_DKF = kron(eye(n_agents), A); % build the A matrix for the DKF
B_DKF = kron(eye(n_agents), B); % build the B matrix for the DKF

for i=1:n_agents
  Q_DKF = blkdiag(Q_DKF, agents{i}.Q); % build the Q matrix for the DKF
end

% Prediction
for i=1:n_agents
  x_est{i} = reshape(agents{i}.x, n_agents*states_len, 1);
  x_est{i} = A_DKF*x_est{i} + B_DKF*u_DKF; % predict the state

  P{i} = A_DKF*P{i}*A_DKF' + B_DKF*Q_DKF*B_DKF'; % predict the covariance
end

% Update
F = cell(n_agents,1);
a = cell(n_agents,1);
for i = 1:n_agents
  for j = 1:n_agents
    if i ~= j
      dist3D = norm(agents{i}.x_real - agents{j}.x_real); % distance between robots in 3D space
      % add a robot in the neightbours set only if it is inside in the communication range and if it is at the seme height
      if dist3D <= agents{i}.Rc
        rel_mes = (agents{j}.x_real - agents{i}.x_real) + mvnrnd(zeros(states_len, 1), agents{i}.R_relative)'; % perform the relative measure as the real distance between the agents plus a noise

        abs_mes = agents{i}.x(1:3, i) + rel_mes; % Project the relative meas in abs.
        zi{i}((j-1)*states_len+1:j*states_len) = abs_mes; % position of the agents j known by i
        
        % Propagate the uncertainty on the relative measurement
        H = eye(measure_len, measure_len); % model of the relative measurement
        P{i}((i-1)*states_len+1:i*states_len,(j-1)*states_len+1:j*states_len) = H*P{i}((i-1)*states_len+1:i*states_len,(j-1)*states_len+1:j*states_len)*H' + agents{i}.R_relative;
        
      else % if one agent does not see another, then it assumes that the other agents is further than twice the communication range, and it also sets the covariance of the estimation to a high value 
          
        zi{i}((j-1)*states_len+1:j*states_len) = zeros(states_len, 1); % position of the agents j known by i
        P{i}((i-1)*states_len+1:i*states_len,(j-1)*states_len+1:j*states_len) = P_est_init*eye(states_len, states_len); 
      end
    end
  end 
  % uncertainty on the sensor readings Ri{i} + Hi{i}*P_est{i}*Hi{i}'
  F{i} = agents{i}.H'*inv(P{i})*agents{i}.H;
  a{i} = agents{i}.H'*inv(P{i})*zi{i}';
end

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
end

%% Measurements update
for i = 1:n_agents
  PredP = P{i};
  P{i} = inv(P{i} + n_agents*F{i});
  x_est{i} = P{i}*(PredP*x_est{i} + n_agents*a{i});
end

%% Extract the estimates
for i = 1:n_agents
  for j = 1:n_agents
    agents{i}.x(1:3, j) = x_est{i}((j-1)*states_len+1:j*states_len);
    agents{i}.P_est{j}(1:3, 1:3) = P{i}((i-1)*states_len+1:i*states_len,(j-1)*states_len+1:j*states_len);
  end
end


% for i = 1:n_agents

%   % previous step state estimation
%   x_est = agents{i}.x_i_previous(1:3); 
%   % previous step covariance estimation
%   P_est = agents{i}.P_est_previous; 
%   % simulate the GPS measure
%   z_GPS = agents{i}.x_real(1:3) + mvnrnd(zeros(3, 1), agents{i}.R_GPS)'; 
%   % GPS covariance 
%   R_GPS = agents{i}.R_GPS;  
%   % external disturbance
%   nu = zeros(4,1);
%   nu(:) = agents{i}.nu;
%   % control input noise covariance matrix
%   Q = agents{i}.Q;        
%   % model of the GPS
%   H_GPS = agents{i}.H_GPS;  
%   % noise covariance matrix
%   L = agents{i}.L;         
  
%   % input with noise 
%   u_bar = agents{i}.u + mvnrnd(zeros(inputs_len, 1), Q)'; 

%   [x_est, P_est] = kalman_filter_chute(x_est, P_est, z_GPS, R_GPS, A, B, G, u_bar, nu, Q, H_GPS, L, states_len); % perform the kalman filter
%   agents{i}.x(1:3, i) = x_est(1:3); % update the estimate position
%   agents{i}.x_i_previous(1:3) = x_est(1:3); % estimate position before the WLS
%   agents{i}.P_est_previous = P_est;         % estimated covariance before the WLS
%   agents{i}.P_est{i}(1:3, 1:3) = P_est(1:3, 1:3); % update the covariance of the estimate position

%    % update the estimate position with the true one
% %   agents{i}.x(1:3, i) = agents{i}.x_real(1:3);

% end

% %% Simulate the communication between agents
% for i = 1:n_agents
%   for j = 1:n_agents
%     if i ~= j
%       dist3D = norm(agents{i}.x_real - agents{j}.x_real); % distance between robots in 3D space
%       % add a robot in the neightbours set only if it is inside in the communication range and if it is at the seme height
%       if dist3D <= agents{i}.Rc
%         rel_mes = (agents{j}.x_real - agents{i}.x_real) + mvnrnd(zeros(states_len, 1), agents{i}.R_relative)'; % perform the relative measure as the real distance between the agents plus a noise

%         abs_mes = agents{i}.x(1:3, i) + rel_mes; % Project the relative meas in abs.
%         agents{i}.x(1:3, j) = abs_mes; % position of the agents j known by i

%         % Propagate the uncertainty on the relative measurement
%         H = eye(measure_len, measure_len); % model of the relative measurement
%         agents{i}.P_est{j}(1:3, 1:3) = H*agents{i}.P_est{i}(1:3, 1:3)*H' + agents{i}.R_relative;
%       else % if one agent does not see another, then it assumes that the other agents is further than twice the communication range, and it also sets the covariance of the estimation to a high value 
%         agents{i}.x(1:2, j) = target(1:2);
%         agents{i}.x(3, j) = 5*x0(3);
%         agents{i}.P_est{j} = P_est_init*eye(states_len, states_len);    
%       end
%     end
%   end
% end