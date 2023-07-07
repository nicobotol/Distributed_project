function [agents] = low_level_control_chute(agents, t)
%% This function computes the low level control u

parameters
for i=1:n_agents
  if agents{i}.x(3, i) > target(3) % Check if we have touched the ground
    %% Low level control
    
    switch mdl
    case 2
      agents{i}.u = agents{i}.kp*(agents{i}.centroid(1:inputs_len) - agents{i}.x(1:inputs_len, i)); % low level control
    case 4
      error('Not implemented yet')
    case 6  
      agents{i}.u = agents{i}.kp*(agents{i}.centroid(1:inputs_len-1) - agents{i}.x(1:inputs_len-1, i)); % low level control
      if agents{i}.x(3,i) < ground_th % if the agent is near the ground, we apply the minimum constant input on the z coordinate of the centroid
        agents{i}.u(3) = agents{i}.vz_min - agents{i}.nu(4);
      else
        % compute the control input and constrain it between the upper and lower bounds 
        % u_cmp = kp_z/(agents{i}.x(3,i) - agents{i}.z_min) - kp_z/agents{i}.Rsv; % positive value
        kp_z = - agents{i}.nu(4)/agents{i}.Rsv;
        u_cmp = - agents{i}.nu(4) - kp_z*(agents{i}.x(3,i) - agents{i}.z_min); 
        v_tmp = min(agents{i}.vz_min, agents{i}.nu(4)); % physical limitation: is the minimum between the free falling (which is a time-dependent velocity) and the minimum velocity at which the chute can fly
        agents{i}.u(3) = min(u_cmp, agents{i}.vz_min - v_tmp); % set the control velocity. Initilly you cannot breake because you are slower (in magnitude) than the minimum velocity at which you can arrive while braking
      end
    end

    % limit the velocities to respect the boundaries
    if agents{i}.u(1) <= 0
      agents{i}.u(1) =  max(agents{i}.u(1), -agents{i}.vmax);
    else
      agents{i}.u(1) =  min(agents{i}.u(1), agents{i}.vmax);
    end
    if agents{i}.u(2) <= 0
      agents{i}.u(2) =  max(agents{i}.u(2), -agents{i}.vmax);
    else
      agents{i}.u(2) =  min(agents{i}.u(2), agents{i}.vmax);
    end


    %% Update the state of the agent
    % input with external disturbance
    agents{i}.u_bar = agents{i}.u + mvnrnd(zeros(inputs_len, 1)', agents{i}.Q)';
    if agents{i}.u_bar(1) <= 0
      agents{i}.u_bar(1) =  max(agents{i}.u_bar(1), -agents{i}.vmax);
    else
      agents{i}.u_bar(1) =  min(agents{i}.u_bar(1), agents{i}.vmax);
    end
    if agents{i}.u_bar(2) <= 0
      agents{i}.u_bar(2) =  max(agents{i}.u_bar(2), -agents{i}.vmax);
    else
      agents{i}.u_bar(2) =  min(agents{i}.u_bar(2), agents{i}.vmax);
    end
    if agents{i}.u_bar(3) < 0
      agents{i}.u_bar(3) = 0;
    end
    if agents{i}.u_bar(3) > 0
      v_tmp = min(agents{i}.vz_min, agents{i}.nu(4)); % physical limitation: is the minimum between the free falling (which is a time-dependent velocity) and the minimum velocity at which the chute can fly
      agents{i}.u_bar(3) = min(agents{i}.u_bar(3), agents{i}.vz_min - v_tmp);
    end

  else % we have touched the ground
    agents{i}.u = zeros(inputs_len, 1);
  end
end
end