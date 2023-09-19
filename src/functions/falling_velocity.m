function vz = falling_velocity(agents, v_lim, v_free_falling, Beta, dt, t)
  %% This function computes the falling velocity of a mass in a fluid
  % t -> time instant at which the velocity is computed (it is dimensionless)
  
  % if agents.terminal_speed == 0
  %     vz = -v_free_falling*(1-exp(-Beta*dt*t));
  % else
      vz = -v_lim;
  % end
  
  end