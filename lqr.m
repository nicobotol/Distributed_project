function K = lqr(A, B, S, R, T, Sf, n, states_len)

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
for i=1:n
  P{i} = zeros(states_len, states_len, T);
  P{i}(:, :, T) = Sf;
  % LQR algorithm
  for j=T:-1:2
    P{i}(:,:,j-1) = S + A'*P{i}(:,:,j)*A - A'*P{i}(:,:,j)*B*inv(R + ...
      B'*P{i}(:,:,j)*B)*B'*P{i}(:,:,j)*A;
  end
end

% Gain matrix calculation
for t=1:T-1
  for i=1:n
  K{i}(:,:,t) = inv(R + B'*P{i}(:, :, t + 1)*B)*B'*P{i}(:, :, t + 1)*A; 
  end
end
  
end