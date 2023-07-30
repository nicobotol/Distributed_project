function agents =  external_disturbance_chutes(agents, t)
%% This function computes the external disturbance nu

parameters;

for i=1:n_agents
  if agents{i}.x_real(3) > target(3)
    % external disturbance
    agents{i}.nu(:) = mvnrnd(zeros(size(agents{i}.nu, 1), 1), agents{i}.L)';   
    switch mdl
      case 2
        agents{i}.nu(4) = -V_z;
      case 4
        error('Not implemented yet')
      case 6  
        agents{i}.nu(4) = falling_velocity(v_lim, Beta, dt, t); % falling velocity of a mass in a fluid
    end
  else
    agents{i}.nu = zeros(4,1);
  end

end