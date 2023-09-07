function agents = free_falling(agents, t, par)
% This function computes the chutes dynamic while they're still reaching
% the terminal velocity before opening

n_agents = par.n_agents;

for i=1:n_agents
    % check if the falling velocity has converged to a stable value
    if abs(agents{i}.vz - agents{i}.vz_old) < 1e-2 && agents{i}.vz ~= 0
        agents{i}.terminal_speed = 1;
        if agents{i}.t_falling == 0 
            agents{i}.t_falling = t;
        end
    else
        agents{i}.terminal_speed = 0;
    end
    
    agents{i}.vz_old = agents{i}.vz;
end

end
