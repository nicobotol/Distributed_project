function [x_est, P_est] = kalman_filter(x_est, P_est, z, R, A, B, G, u_bar, Q, H, L, t, states_len)
% This function implements the Kalman filter
% x_est -> estimation of the state
% P_est -> estimation of the covariance matrix
% z -> measurement
% R -> measurement noise covariance matrix
% A -> state transition matrix
% B -> control input matrix
% G -> model of the noise
% u_bar -> control input (with noise)
% Q -> control input noise covariance matrix
% H -> sensor model
% L -> noise covariance matrix
% t -> time step
% states_len -> number of states

% number of agents
n = length(x_est);

for i=1:n

  % Prediction
  x_est{i}(:, t+1) = A*x_est{i}(:, t) + B*u_bar{i}(:, t); % CAMBIARE U_BAR CON U, PERCHÉ NON CONOSCIAMO LA VERA DINAMICA. U_BAR VA USATO NELL'AGGIORNAMENTO DELLO STATO
  P_est{i} = A*P_est{i}*A' + B*Q{i}*B' + G*L{i}*G';

  % Measurement update using the GPS
  Innovation{i} = z(:, i) - x_est{i}(:, t + 1);
  % update the kalaman estimate
  S_Inno{i} = H{i}*P_est{i}*H{i}' + R{i};
  W{i} = P_est{i}*H{i}'*inv(S_Inno{i}); % kalman gain
  x_est{i}(:, t + 1) = x_est{i}(:, t + 1) + W{i}*Innovation{i}; % update stte estimate
  P_est{i} = (eye(states_len) - W{i}*H{i})*P_est{i}; % update covariance matrix

end

end