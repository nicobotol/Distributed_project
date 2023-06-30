function [] = check_correlation(agents)

% This function test the correlation bewteen the state of the output
parameters

%% Check the correlation
x_reshape = zeros(3*n_samples, n_agents);
for i=1:n_agents
  noise = normrnd(0, 1, 3, n_samples); % gaussian noise
  x_noise = agents{i}.x(:, i) + noise; % add noise to the state
  x_reshape(:, i) = reshape(x_noise, 3*n_samples, 1); % reshape the state in a vector
end
corrcoef(x_reshape) % compute the correlation matrix

end