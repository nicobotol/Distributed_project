function [x_est, P_est] = extended_kalman_filter_chute(x_est, P_est, z, R, A, B, G, u, nu, Q, H, L, states_len)
% This function implements the Extended Kalman Filter
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
% A_lin -> linearized state transition matrix (derivative of dynamic wrt states)
% B_lin -> linearized control input matrix (derivative of dynamic wrt )

% Prediction
x_est = A*x_est + B*u + G*[0;0;0;nu(4)]; % propagate the state with the NL function
P_est = A_lin*P_est*A_lin' + B_lin*Q*B_lin';

% Measurement update using the GPS
Innovation = z - x_est;
% update the kalaman estimate
S_Inno = H*P_est*H' + R;
W = P_est*H'*inv(S_Inno); % kalman gain
x_est = x_est + W*Innovation; % update state estimate
P_est = (eye(states_len) - W*H)*P_est; % update covariance matrix


end