function par = set_parameters(k, variable_param, param)
  
  switch param
    case 'prob_GPS'
      variable_param.prob_GPS = k;
    case 'prob_conn'
      variable_param.prob_conn = k;
    case 'prob_rel_measurement'
      variable_param.prob_rel_measurement = k;
    otherwise 
      error('Case not implemented yet!')
  end

  par = parameters(variable_param);

  % Set to 1 the parameters not investigated
  switch param
    case 'prob_GPS'
      par.prob_conn = 1;
      par.prob_rel_measurement = 1;
    case 'prob_conn'
      par.prob_GPS = 1;
      par.prob_rel_measurement = 1;
    case 'prob_rel_measurement'
      par.prob_GPS = 1;
      par.prob_conn = 1;
    otherwise 
      error('Case not implemented yet!')
  end

end