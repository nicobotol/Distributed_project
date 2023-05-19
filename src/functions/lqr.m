function K = lqr(A, B, S, R, T, Sf, states_len)

% A -> state matrix
% B -> input matrix
% minimization function: cost = x_f*Sf*x_f + sum(i=1:N) (x(:,i)-target)*S*(x(:,i)-target)' + u(:,i)*R*u(:,i)'
% S -> cost state matrix
% Sf -> final cost state matrix
% R -> cost input matrix
% T -> time horizon
% n -> number of agents

input_len = size(B, 2); % number of inputs
K = zeros(input_len, states_len, T);

% Backward cycle
P = zeros(states_len, states_len, T);
P(:, :, T) = Sf;
% LQR algorithm
for j=T:-1:2
  P(:,:,j-1) = S + A'*P(:,:,j)*A - A'*P(:,:,j)*B*inv(R + B'*P(:,:,j)*B)*B'*P(:,:,j)*A;
end

% Gain matrix calculation
for t=1:T-1
  K(:,:,t) = inv(R + B'*P(:, :, t + 1)*B)*B'*P(:, :, t + 1)*A;
end

end