function vz = falling_velocity(v_lim, Beta, dt, t)
%% This function computes the falling velocity of a mass in a fluid
% t -> time instant at which the velocity is computed (it is dimensionless)

  vz = -v_lim*(1-exp(-Beta*dt*t));

end