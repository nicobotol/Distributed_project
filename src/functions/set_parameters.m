function par = set_parameters(k, variable_param, param)

switch param
  case 'prob_GPS'
    variable_param.prob_GPS = k;
  case 'prob_conn'
    variable_param.prob_conn = k;
  case 'prob_rel_measuremnt'
    variable_param.prob_rel_measuremnt = k;
  otherwise
    error('Case not implemented yet!')
end

par = parameters(variable_param);