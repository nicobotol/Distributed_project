function agents = localization_chutes(agents)
% Localize the chutes and perform the relative measurements

parameters; % load the parameters

n_agents = length(agents);

% Localize the chute TO BE UPDATED FOR THE MODEL WITH THETA
% agents{i}.x contains the state after the reaching of the consensus on the previous step, and so agents{i}.x(:, i) depends also on the KFs of the other N-1 agents, and so it cannot be used as estimation for a further step of the KF. For this reason the pure estimation of the position is stored in a separate filed and used for the update at the next step
for i = 1:n_agents

  % % previous step state estimation
  % x_est = agents{i}.x_i_previous(1:3); 
  % % previous step covariance estimation
  % P_est = agents{i}.P_est{i}(1:3, 1:3); 
  % % simulate the GPS measure
  % z_GPS = agents{i}.x_real(1:3) + mvnrnd(zeros(3, 1), agents{i}.R_GPS)'; 
  % % GPS covariance 
  % R_GPS = agents{i}.R_GPS;  
  % % external disturbance
  % nu = zeros(4,1);
  % nu(:) = agents{i}.nu(4);
  % % control input noise covariance matrix
  % Q = agents{i}.Q;        
  % % model of the GPS
  % H_GPS = agents{i}.H_GPS;  
  % % noise covariance matrix
  % L = agents{i}.L;         
  
  % % input with noise 
  % u_bar = agents{i}.u + mvnrnd(zeros(inputs_len, 1), Q)'; 

  % [x_est, P_est] = kalman_filter_chute(x_est, P_est, z_GPS, R_GPS, A, B, G, u_bar, nu, Q, H_GPS, L, states_len); % perform the kalman filter
  % agents{i}.x(1:3, i) = x_est(1:3); % update the estimate position
  % agents{i}.x_i_previous(1:3) = x_est(1:3); % update the previous estimate position
  % agents{i}.P_est{i}(1:3, 1:3) = P_est(1:3, 1:3); % update the covariance of the estimate position

  agents{i}.x(1:3, i) = agents{i}.x_real;
end

%% Simulate the communication between agents
for i = 1:n_agents
  for j = 1:n_agents
    if i ~= j
      dist3D = norm(agents{i}.x_real - agents{j}.x_real); % distance between robots in 3D space
      % add a robot in the neightbours set only if it is inside in the communication range and if it is at the seme height
      if dist3D <= agents{i}.Rc
        rel_mes = (agents{j}.x_real - agents{i}.x_real) + mvnrnd(zeros(inputs_len, 1), agents{i}.R_relative)'; % perform the relative measure as the real distance between the agents plus a noise

        abs_mes = agents{i}.x(1:3, i) + rel_mes; % Project the relative meas in abs.
        agents{i}.x(1:3, j) = abs_mes; % position of the agents j known by i

        % Propagate the uncertainty on the relative measurement
        agents{i}.P_est{j}(1:3, 1:3) = agents{i}.P_est{i}(1:3, 1:3) + agents{i}.R_relative;
      else % if one agent does not see another, then it assumes that the other agents is further than twice the communication range, and it also sets the covariance of the estimation to a high value 
        agents{i}.x(1:3, j) = agents{i}.x(1:3, i) + 5*(1 + rand([3, 1]))*agents{i}.Rc;
        agents{i}.P_est{j} = P_est_init*eye(states_len, states_len);    
      end
    end
  end
end