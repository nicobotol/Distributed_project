function [agents, ground_check] = dynamic_chutes(agents, ground_check, dt)
% This function prpagates the dynamic of the agents given the input and the noises

parameters;

for i=1:n_agents
  if ground_check(i) == 1
    agents{i}.u_bar = zeros(inputs_len, 1);
    agents{i}.nu = zeros(nc_inputs_len, 1);
  end

  % propagate the dynamic with the inputs
  if mdl == 5 % the model is the unicycle
    agents{i}.x_real = unicycle_dynamics(agents{i}.x_real, agents{i}.u_bar, agents{i}.nu, dt);
    agents{i}.x_real(4) = wrapTo2Pi(agents{i}.x_real(4));
    if agents{i}.x_real(3) == 2*pi
      agents{i}.x_real(3) = 0;
    end
  else % the model is linear 
    agents{i}.x_real = A*agents{i}.x_real + B*agents{i}.u_bar + G*agents{i}.nu;
  end

  if agents{i}.x_real(3) <= target(3) 
    agents{i}.x_real(3) = 0;
    ground_check(i) = 1; % mark that the agent has touched the ground
  end
  agents{i}.x_real_store = [agents{i}.x_real_store, agents{i}.x_real];   % save the history of the agent's real state

end

end