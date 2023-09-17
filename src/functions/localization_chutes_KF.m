function agents = localization_chutes_KF(agents, ground_check, t, par)
% Localize each chute with its own KF combining GPS and relative measurements

  mdl = par.mdl;
  measure_len = par.measure_len;
  states_len = par.states_len;
  P_est_init = par.P_est_init;
  A = par.A;
  B = par.B;
  if par.mdl == 1
    G = par.G;
  else 
    G = eye(3);
  end
  dt = par.dt;
  v_lim = par.v_lim;
  v_free_falling = par.v_free_falling;
  Beta = par.Beta;
  prob_rel_measurement = par.prob_rel_measurement;
  coverage_dropout = par.coverage_dropout;
  prob_GPS = par.prob_GPS;
  

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
    nu = agents{i}.nu;   
    
    % input with noise 
    u = agents{i}.u; 

    if mdl == 1
      [x_est, P_est] = kalman_filter_chute(x_est, P_est, z_GPS, R_GPS, A, B, G, u, nu, Q, H_GPS, states_len, prob_GPS, agents{i}.L(1:3,1:3)); % perform the kalman filter
    else
      nu = zeros(5, 1); % no noise in the dynamics
      if ground_check(i) == 0 
        nu(4) = falling_velocity(agents{i}, v_lim, v_free_falling, Beta, dt, t); % add the falling velocity
      end
      [x_est, P_est] = extended_kalman_filter_chute(x_est, P_est, z_GPS, nu, R_GPS, u, Q, H_GPS, states_len, dt, prob_GPS, agents{i}.L);
    end


    % Check to be above ground
    if x_est(3) <= 0
      agents{i}.x(3, i) = 0;
    end
    agents{i}.x(:, i) = x_est(:); % update the estimate position
    agents{i}.x_store = [agents{i}.x_store, x_est];       % save the history of the agent's state
    agents{i}.x_i_previous = x_est; % estimate position before the WLS
    agents{i}.P_est_previous = P_est;         % estimated covariance before the WLS
    agents{i}.P_est{i} = P_est; % update the covariance of the estimate position

  end

  %% Simulate the communication between agents
  for i = 1:n_agents
    for j = 1:n_agents
      if i ~= j
        dist = norm(agents{i}.x_real(1:2) - agents{j}.x_real(1:2)); % distance between robots in 2D space
        dist_z = norm(agents{i}.x_real(3) - agents{j}.x_real(3)); % distance between robots in the vertical direction

        % add a robot in the neightbours set only if it is inside its planar communication range, it is reasonably close in the vertical direction, and consider also the probability of communication
        if dist <= agents{i}.Rc && dist_z <= agents{i}.Rcv && rand(1) <= prob_rel_measurement
          rel_mes = (agents{j}.x_real(1:measure_len) - agents{i}.x_real(1:measure_len)) + mvnrnd(zeros(measure_len, 1), agents{i}.R_relative)'; % perform the relative measure as the real distance between the agents plus a noise

          abs_mes = agents{i}.x(1:3, i) + rel_mes; % Project the relative meas in abs.
          agents{i}.x(1:3, j) = abs_mes; % position of the agents j known by i

          % Propagate the uncertainty on the relative measurement
          H = eye(measure_len, measure_len); % model of the relative measurement
          agents{i}.P_est{j}(1:3, 1:3) = H*agents{i}.P_est{i}(1:3, 1:3)*H' + agents{i}.R_relative;

          % Add the chute to the register of visitations
          if ismember(j, agents{i}.visited_chutes) == 0
            agents{i}.visited_chutes = [agents{i}.visited_chutes, j];
          end 
          agents{i}.u_visit(:,j) = agents{j}.u;
          % if j belongs to the regsiter and the covariance of the estimation is not too high, then the agent i can propagate the state of the agent j
        elseif ismember(j, agents{i}.visited_chutes) == 1 && max([sqrt(agents{i}.P_est{j}(1,1)), sqrt(agents{i}.P_est{j}(2,2)), sqrt(agents{i}.P_est{j}(3,3))]) < coverage_dropout*max([sqrt(agents{i}.P_est{i}(1,1)), sqrt(agents{i}.P_est{i}(2,2)), sqrt(agents{i}.P_est{i}(3,3))])
          % propagate the state using as input the last control of the agent j
          if mdl == 1
            agents{i}.x(1:2, j) = A(1:2,1:2)*agents{i}.x(1:2, j) + B(1:2,1:2)*agents{i}.u_visit(1:2,j);
            agents{i}.x(3,j) = A(3,3)*agents{i}.x(3,j) + B(3,3)*agents{i}.u_visit(3,j) + G(3,4)*agents{i}.nu(4);
          elseif mdl == 2
            prediction = unicycle_dynamics(agents{i}.x(:, j), agents{i}.u_visit(:, j), [0 0 0 agents{i}.nu(4) 0]', dt);
            agents{i}.x(1:3, j) = prediction(1:3);
          else 
            fprintf('Model not implemented\n');
            break
          end

          agents{i}.P_est{j}(1:3, 1:3) = A*agents{i}.P_est{j}*A' + B*agents{i}.Q*B' + G(1:3,1:3)*agents{i}.L(1:3,1:3)*G(1:3,1:3)';
        else 
          if ismember(j, agents{i}.visited_chutes) == 1
            agents{i}.visited_chutes = agents{i}.visited_chutes(agents{i}.visited_chutes ~= j);
          end
          % if one agent does not see another, then it assumes that the other agents is further than 50 times the current position of its position
          agents{i}.x(1:2, j) = agents{i}.x(1:2,i)*50;
          agents{i}.x(3, j) = agents{i}.x(3,i)*50;
          agents{i}.P_est{j} = P_est_init*eye(states_len, states_len);    
        end
      end
    end
  end

end