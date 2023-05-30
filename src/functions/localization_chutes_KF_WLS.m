function agents = localization_chutes_KF_WLS(agents)
% Localize each chute with its own KF combining GPS and relative measurements, then distribute the informations with a WLS

parameters; % load the parameters

n_agents = length(agents);

% Localize the chute TO BE UPDATED FOR THE MODEL WITH THETA
% agents{i}.x contains the state after the reaching of the consensus on the previous step, and so agents{i}.x(:, i) depends also on the KFs of the other N-1 agents, and so it cannot be used as estimation for a further step of the KF. For this reason the pure estimation of the position is stored in a separate filed and used for the update at the next step
for i = 1:n_agents

  % previous step state estimation
  x_est = agents{i}.x_i_previous; 
  % previous step covariance estimation
  P_est = agents{i}.P_est_previous; 
  % simulate the GPS measure
  z_GPS = agents{i}.x_real + mvnrnd(zeros(states_len, 1), agents{i}.R_GPS)'; 
  % GPS covariance 
  R_GPS = agents{i}.R_GPS;  

  % control input noise covariance matrix
  Q = agents{i}.Q;        
  % model of the GPS
  H_GPS = agents{i}.H_GPS;  
  % noise covariance matrix
  L = agents{i}.L;         
  
  % input with noise 
  u_bar = agents{i}.u + mvnrnd(zeros(inputs_len, 1), Q)'; 
  nu = agents{i}.nu;
  if mdl == 4
    G_est = G(agents{i}.x(4,i));
  end
  
  [x_est, P_est] = kalman_filter_chute(x_est, P_est, z_GPS, R_GPS, A, B, G_est, u_bar, nu, Q, H_GPS, L, states_len); % perform the kalman filter
  agents{i}.x(1:3, i) = x_est(1:3); % update the estimate position
  agents{i}.x_i_previous(1:3) = x_est(1:3); % estimate position before the WLS
  agents{i}.P_est_previous = P_est;         % estimated covariance before the WLS
  agents{i}.P_est{i}(1:3, 1:3) = P_est(1:3, 1:3); % update the covariance of the estimate position

   % update the estimate position with the true one
%   agents{i}.x(1:3, i) = agents{i}.x_real(1:3);

end

%% Simulate the communication between agents
for i = 1:n_agents
  for j = 1:n_agents
    if i ~= j
      dist3D = norm(agents{i}.x_real - agents{j}.x_real); % distance between robots in 3D space
      % add a robot in the neightbours set only if it is inside in the communication range and if it is at the seme height
      if dist3D <= agents{i}.Rc
        rel_mes = (agents{j}.x_real(1:measure_len) - agents{i}.x_real(1:measure_len)) + mvnrnd(zeros(measure_len, 1), agents{i}.R_relative)'; % perform the relative measure as the real distance between the agents plus a noise

        abs_mes = agents{i}.x(1:3, i) + rel_mes; % Project the relative meas in abs.
        agents{i}.x(1:3, j) = abs_mes; % position of the agents j known by i

        % Propagate the uncertainty on the relative measurement
        H = eye(measure_len, measure_len); % model of the relative measurement
        agents{i}.P_est{j}(1:3, 1:3) = H*agents{i}.P_est{i}(1:3, 1:3)*H' + agents{i}.R_relative;
      else % if one agent does not see another, then it assumes that the other agents is further than twice the communication range, and it also sets the covariance of the estimation to a high value 
        agents{i}.x(1:2, j) = target(1:2);
        agents{i}.x(3, j) = 5*x0(3);
        agents{i}.P_est{j} = P_est_init*eye(states_len, states_len);    
      end
    end
  end

  % Distribute the informations about the position between agents
  agents = distribute_informations2(agents);
end