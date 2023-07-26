function [sim_x, w] = inverse_kinematics(agent, i, u_global_centroid, dt, K, states_len, A, B, t)
  %% This function computes the target point of the agents (i.e. where do we want that they go) given the new position of the global centroid. The new agent's position takes into account both the final taret of the centroid but also a possible "postural task"

  
% Propagte the position of the global centroid
centroid_d = eye(2)*agent.global_centroid(1:2) + dt*eye(2)*u_global_centroid;

% Set the number of agent seen by the current one
if length(agent.x_idx) < 2 % In case of an agent seeing only itself and another
  x_idx = [i, agent.x_idx]; % index of the agents seen by the current agent plus the agent itself
else
  x_idx = agent.x_idx; % index of the agents seen by the current agent
end

seen_agents = length(x_idx); % number of agents seen by the current agent
x_d = zeros(states_len, seen_agents); % desired position of the agents seen by the current agent

% Compute the desired position of all the agents seen
for j=1:seen_agents
  % Position of the agent
  x(:, j) = agent.x(1:2, x_idx(j));

  % Compute agent input
  u_tmp = -K(1:2,1:2)*(x(:, j) - centroid_d);

  % Propagate the position 
  x_d(1:2, j) = A(1:2,1:2)*x(:,j) + B(1:2,1:2)*u_tmp;
end

% Compute the jacobian matrix [states_len, states_len*seen_agents]
% w = 1e-3;
% w = 0.2*2/pi*atan(t) + 1e-3;
w = 1e-3/0.5*((1/(1+exp((-0.1*t))))-0.5);
gamma = 1e-3;
beta = 1e-3;
j1 = 1/seen_agents*eye(states_len); 
J = kron(ones(1, seen_agents), j1);
J_bar = [J; w*eye(states_len*seen_agents)];

% Compute centroid and agents errors
centroid = mean(x,2);
e_c = [centroid_d - centroid];
e_x = [];
for j=1:seen_agents
  e_x_tmp = [x_d(:, j) - x(:, j)];
  e_x = [e_x; e_x_tmp];
end
e = [e_c; e_x];


while (norm(e_c)^2 >= 0.1) && (norm(e_x)^2 >= 0.5)
  alpha = 1;
  delta_x = inv(J'*J + beta^2*eye(states_len*seen_agents))*(J'*e_c + w^2*e_x);
  delta_x = reshape(delta_x,states_len,seen_agents);
  x_test = x + alpha*delta_x;
  centroid = mean(x_test,2);
  e_c_test = [centroid_d - centroid];
  e_x_test = [];
  for j=1:seen_agents
    e_x_test_tmp = [x_d(:, j) - x_test(:,j)];
    e_x_test = [e_x_test; e_x_test_tmp];
  end
  e_test = [e_c_test;e_x_test];
  red_err = norm(e) - norm(e_test);
  while red_err - gamma*alpha*norm(e) < 1e-10
    alpha = alpha/2;
    x_test = x + alpha*delta_x;
    centroid = mean(x_test,2);
    e_c_test = [centroid_d - centroid];
    e_x_test = [];
    for j=1:seen_agents
      e_x_test_tmp = [x_d(:, j) - x_test(:,j)];
      e_x_test = [e_x_test; e_x_test_tmp];
    end
    e_test = [e_c_test;e_x_test];
    red_err = norm(e) - norm(e_test);
  end
  x = x_test;
  e = e_test;
  e_c = [centroid_d - centroid];
  e_x = [];
  for j=1:seen_agents
    e_x_tmp = [x_d(:,j) - x(:,j)];
    e_x = [e_x; e_x_tmp];
  end
  J_bar = [J;w*eye(states_len*seen_agents)];
end

% Extract final agents positions
sim_x = x(:, 1);

end