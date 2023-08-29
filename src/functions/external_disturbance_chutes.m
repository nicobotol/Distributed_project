function agents =  external_disturbance_chutes(agents, t, par)
%% This function computes the external disturbance nu
target = par.target;
mdl = par.mdl;
n_agents = par.n_agents;
V_z = par.V_z;
v_lim = par.v_lim;
Beta = par.Beta;
dt = par.dt;

for i=1:n_agents
  if agents{i}.x_real(3) > target(3)
    % external disturbance
    agents{i}.nu(:) = mvnrnd(zeros(size(agents{i}.nu, 1), 1), agents{i}.L)'; 
    switch mdl
      case 2
        agents{i}.nu(4) = -V_z;
      case 4
        error('Not implemented yet')
      case {5, 6}
        agents{i}.nu(4) = falling_velocity(v_lim, Beta, dt, t); % falling velocity of a mass in a fluid
    end
  else
    switch mdl
    case 5 
      agents{i}.nu = zeros(5,1);
    case 6 
      agents{i}.nu = zeros(4,1);
    end
      
  end

end