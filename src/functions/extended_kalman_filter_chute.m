function [x_est, P_est] = extended_kalman_filter_chute(x_est, P_est, z, nu, R, u, Q, H, states_len, Beta, v_lim, t, dt, prob_GPS)
% This function implements the Extended Kalman Filter
% x_est -> estimation of the state
% P_est -> estimation of the covariance matrix
% A_lin -> linearization of the dynamic wrt the states
% B_lin -> linearization of the dynamic wrt the variables with uncertainty
% z -> measurement
% R -> measurement noise covariance matrix
% u -> control input (without noise)
% Q -> control input noise covariance matrix
% H -> sensor model
% states_len -> number of states
% dt -> time step
% A_lin -> linearized state transition matrix (derivative of dynamic wrt states)
% B_lin -> linearized control input matrix (derivative of dynamic wrt )
% GPS -> 1 if we have the GPS measurement, 0 otherwise

% Prediction
x_est = unicycle_dynamics(x_est, u, nu, dt); % propagate the state with the NL function
x_est(4) = wrapTo2Pi(x_est(4)); % wrap the angle between 0 and 2pi

% A_linear = eval(A_lin(u(1), x_est(4))); % linearized state transition matrix
% B_linear = eval(B_lin(x_est(4)));       % linearized control input matrix
A_linear= [1 0 0 -sin(x_est(4))*u(1)*dt; 
                      0 1 0 cos(x_est(4))*u(1)*dt;
                      0 0 1 0;
                      0 0 0 1];
B_linear = [cos(x_est(4))*dt 0 0;
                  sin(x_est(4))*dt 0 0;
                  0 0 dt;
                  0 dt 0];
P_est = A_linear*P_est*A_linear' + B_linear*Q*B_linear';

if (rand(1) <= prob_GPS) % if we have the GPS measurement we update the kalmen filter with the measerement, otherwise we propagate the prediction
  % Measurement update using the GPS
  Innovation = z - x_est;
  % update the kalaman estimate
  S_Inno = H*P_est*H' + R;
  W = P_est*H'*inv(S_Inno); % kalman gain
  x_est = x_est + W*Innovation; % update state estimate
  P_est = (eye(states_len) - W*H)*P_est; % update covariance matrix
else
  x_est = x_est;
  P_est = P_est;
end

end