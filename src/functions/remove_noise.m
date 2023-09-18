function [par] = remove_noise(par)

  par.R_GPS_scale = 1e-6;
  par.R_compass_scale = 1e-6;
  par.R_relative = 0;
  par.L_scale = 0;
  par.L_compass_scale = 0;
  if par.mdl == 1
    par.Q_scale_vx = 0;
    par.Q_scale_vy = 0;
    par.Q_scale_vz = 0;
  elseif par.mdl == 2
    par.Q_scale_V = 0;
    par.Q_scale_omega = 0;
    par.Q_scale_vz = 0;
  end

end