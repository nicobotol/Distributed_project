function K = lqr(A, B, S, R, T, Sf, states_len)

% A -> state matrix
% B -> input matrix
% minimization function: cost = x_f*Sf*x_f + sum(i=1:N) (x(:,i)-target)*S*(x(:,i)-target)' + u(:,i)*R*u(:,i)'
% S -> cost state matrix
% Sf -> final cost state matrix
% R -> cost input matrix
% T -> time horizon
% n -> number of agents

P = cell(1, T);
P{T} = Sf;
K = cell(1, T);

% Backward cycle
P{i} = zeros(states_len, states_len, T);
P{i}(:, :, T) = Sf;
% LQR algorithm
for j=T:-1:2
  P(:,:,j-1) = S + A'*P(:,:,j)*A - A'*P(:,:,j)*B*inv(R + B'*P(:,:,j)*B)*B'*P(:,:,j)*A;
end

% Gain matrix calculation
for t=1:T-1
  K(:,:,t) = inv(R + B'*P(:, :, t + 1)*B)*B'*P(:, :, t + 1)*A;
end
  
end