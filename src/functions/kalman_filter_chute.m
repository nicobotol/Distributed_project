function [x_est, P_est] = kalman_filter_chute(x_est, P_est, z, R, A, B, G, u, nu, Q, H, states_len, prob_GPS)
% This function implements the Kalman filter
  % x_est -> estimation of the state
  % P_est -> estimation of the covariance matrix
  % z -> measurement
  % R -> measurement noise covariance matrix
  % A -> state transition matrix
  % B -> control input matrix
  % G -> model of the noise
  % u -> control input (without noise)
  % Q -> control input noise covariance matrix
  % H -> sensor model
  % L -> noise covariance matrix
  % states_len -> number of states
  % GPS -> 1 if we have the GPS measurement, 0 otherwise


  % Prediction
  x_est = A*x_est + B*u + G*[0;0;0;nu(4)];
  P_est = A*P_est*A' + B*Q*B';

  if (rand(1) <= prob_GPS) % if we have the GPS measurement we update the kalmen filter with the measerement, otherwise we propagate the prediction
    % Measurement update using the GPS
    Innovation = z - x_est;
    % update the kalaman estimate
    S_Inno = H*P_est*H' + R;
    W = P_est*H'*inv(S_Inno); % kalman gain
    x_est = x_est + W*Innovation; % update state estimate
    P_est = (eye(states_len) - W*H)*P_est; % update covariance matrix
  end

end