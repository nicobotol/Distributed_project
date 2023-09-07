function K = lqr_finite(A, B, S, R, T, Sf, states_len, input_len, mdl, t)

% A -> state matrix
% B -> input matrix
% minimization function: cost = x_f*Sf*x_f + sum(i=1:N) (x(:,i)-target)*S*(x(:,i)-target)' + u(:,i)*R*u(:,i)'
% S -> cost state matrix
% Sf -> final cost state matrix
% R -> cost input matrix
% T -> time horizon
% n -> number of agents

% % Backward cycle
% P = zeros(states_len, states_len, T+1);
K = zeros(2);
P = zeros(2, 2, T+1);
P(:, :, T+1) = Sf;
% LQR algorithm
for j=T:-1:t
  P(:,:,j) = S + A'*P(:,:,j+1)*A - A'*P(:,:,j+1)*B*inv(R + B'*P(:,:,j+1)*B)*B'*P(:,:,j)*A;
end

% Gain matrix calculation
K = inv(R + B'*P(:, :, t + 1)*B)*B'*P(:, :, t + 1)*A;

end