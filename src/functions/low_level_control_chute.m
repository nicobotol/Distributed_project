function [agents] = low_level_control_chute(agents, t, par)
  %% This function computes the low level control u
  t;
  n_agents = par.n_agents;
  mdl = par.mdl;
  inputs_len = par.inputs_len;
  target = par.target;
  ground_th = par.ground_th;
  if par.mdl == 2
      K_v = par.K_v;
      V_min = par.V_min;
      K_omega = par.K_omega;
      omega_max = par.omega_max;
  end
  V_max = par.V_max;
  
  for i=1:n_agents
    if agents{i}.x(3, i) > target(3) && agents{i}.terminal_speed == 1 % Check if we have touched the ground and if the chute is open

      %% Low level control
      switch mdl
        case 1
          agents{i}.u = agents{i}.kp*(agents{i}.centroid(1:inputs_len-1) - agents{i}.x(1:inputs_len-1, i)); % low level control
          % if the agent is near the ground, we apply the minimum constant input on the z coordinate of the centroid
          if agents{i}.x(3,i) < ground_th 
            agents{i}.u(3) = agents{i}.vz_min - agents{i}.nu(4);
          else
            % compute the control input and constrain it between the upper and lower bounds 
            kp_z = - agents{i}.nu(4)/agents{i}.Rsv;
            u_cmp = - agents{i}.nu(4) - kp_z*(agents{i}.x(3,i) - agents{i}.z_min); % positive value
            % set the control velocity. Initilly you cannot breake because you are slower (in magnitude) than the minimum velocity at which you can arrive while braking
            agents{i}.u(3) = min(u_cmp, agents{i}.vz_min - agents{i}.nu(4)); 
          end
          % limit the velocities to respect the boundaries
          if agents{i}.u(1) <= 0
            agents{i}.u(1) = max(agents{i}.u(1), -agents{i}.V_max);
          else
            agents{i}.u(1) = min(agents{i}.u(1), agents{i}.V_max);
          end
          if agents{i}.u(2) <= 0
            agents{i}.u(2) = max(agents{i}.u(2), -agents{i}.V_max);
          else
            agents{i}.u(2) = min(agents{i}.u(2), agents{i}.V_max);
          end
        case 2 % unicycle model
          y = agents{i}.centroid(1:2)';                                         % local target point
          x = agents{i}.x(1:2, i)';                                             % position of the agent
          theta = agents{i}.x(4, i);                                            % orientation of the agent
          [cone, len_cone, dy] = feedback_motion_prediction_chute(theta, x, y); % find the cone of the motion prediction
    
          % subctract the cone and the voronoi cell
          cone_m_voronoi_cell = subtract(cone, agents{i}.voronoi);
    
          if len_cone ~= -1                                                     % if cone is a polyshape
            inside = cone_m_voronoi_cell.NumRegions == 0;        
          else                                                                  % if cone is a segment
            inside = 1;
          end
    
          iter = 0;
          while (inside == 0)
            iter = iter + 1;
            % If the cone goes outside the voronoi cell, then move the target point closer to the starting one of a quanty equal to how much the cone goes outside
            % radius of the motion of the point
            delta = p_poly_dist(y(1), y(2), agents{i}.voronoi.Vertices(:,1), agents{i}.voronoi.Vertices(:,2)); 
            delta = abs(delta);
            theta2 = atan2(y(2) - x(2), y(1) - x(1)); % direction between the target point and the agent position
            moving_radius = min(delta, dy - delta);
            y = y - moving_radius*[cos(theta2) sin(theta2)];               % new target point
            [cone, ~, dy] = feedback_motion_prediction_chute(theta, x, y); % new cone
          
            cone_m_voronoi_cell = subtract(cone, agents{i}.voronoi);
          
            inside = cone_m_voronoi_cell.NumRegions == 0; 
            if iter > 10000
              error('Not convergence')
            end
          end
          agents{i}.motion_predict = cone;
    
          v_cmp = [cos(theta) sin(theta)]*(y - x)';   % unicycle forward velocity
          % limit the unicycle forward velocity, K_v = 1 so that u is bounded between V_min and V_max
          agents{i}.u(1) = K_v*max(V_min, min(V_max, v_cmp)); 
          % if the agent is near the ground, we apply the minimum constant input on the z coordinate of the centroid
          if agents{i}.x(3,i) < ground_th 
            agents{i}.u(3) = agents{i}.vz_min - agents{i}.nu(4);
          else 
            % compute the control input and constrain it between the upper and lower bounds 
            kp_z = - agents{i}.nu(4)/agents{i}.Rsv;
            u_cmp = - agents{i}.nu(4) - kp_z*(agents{i}.x(3,i) - agents{i}.z_min); % positive value
            % set the control velocity. Initilly you cannot breake because you are slower (in magnitude) than the minimum velocity at which you can arrive while braking
            agents{i}.u(3) = min(u_cmp, agents{i}.vz_min - agents{i}.nu(4)); 
          end
          % angular velocity
          agents{i}.u(2) = K_omega*atan2([-sin(theta) cos(theta)]*(y - x)',[cos(theta) sin(theta)]*(y - x)'); 
    
      end
  
      %% Update the state of the agent
      % input with external disturbance
      agents{i}.u_bar = agents{i}.u + mvnrnd(zeros(inputs_len, 1)', agents{i}.Q)';
      switch mdl
        case 1
          if agents{i}.u_bar(1) <= 0
            agents{i}.u_bar(1) = max(agents{i}.u_bar(1), -agents{i}.V_max);
          else
            agents{i}.u_bar(1) = min(agents{i}.u_bar(1), agents{i}.V_max);
          end
          if agents{i}.u_bar(2) <= 0
            agents{i}.u_bar(2) = max(agents{i}.u_bar(2), -agents{i}.V_max);
          else
            agents{i}.u_bar(2) = min(agents{i}.u_bar(2), agents{i}.V_max);
          end
        case 2
          agents{i}.u_bar(1) = max(V_min, min(V_max, agents{i}.u_bar(1)));          % forward speed
          agents{i}.u_bar(2) = max(-omega_max, min(omega_max, agents{i}.u_bar(2))); % angular speed 
      end
      if agents{i}.u_bar(3) < 0
        agents{i}.u_bar(3) = 0;
      end
      if agents{i}.u_bar(3) > 0
        % physical limitation: is the minimum between the free falling (which is a time-dependent velocity) and the minimum velocity at which the chute can fly
        v_tmp = min(agents{i}.vz_min, agents{i}.nu(4)); 
        agents{i}.u_bar(3) = min(agents{i}.u_bar(3), agents{i}.vz_min - v_tmp);
      end
  
    else % we have touched the ground or the chute is not open
      agents{i}.u = zeros(inputs_len, 1);
    end
  end
  end