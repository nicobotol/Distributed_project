function [x_est, P_est] = kalman_filter_chute(x_est, P_est, z, R, A, B, G, u_bar, nu, Q, H, L, states_len)
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
% states_len -> number of states


% Prediction

x_est = A*x_est + B*u_bar + G*nu;
P_est = A*P_est*A' + B*Q*B' + G*L*G';

% Measurement update using the GPS
Innovation = z - x_est;
% update the kalaman estimate
S_Inno = H*P_est*H' + R;
W = P_est*H'*inv(S_Inno); % kalman gain
x_est = x_est + W*Innovation; % update stte estimate
P_est = (eye(states_len) - W*H)*P_est; % update covariance matrix


end