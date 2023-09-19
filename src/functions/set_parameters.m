function par = set_parameters(k, variable_param, param, user_par,par)
  % This function sets the index to be used for entering in the parameters vector and to decide which is the variable parameter
  
  switch param
    case 'prob_GPS'
      variable_param.prob_GPS = k;
    case 'prob_connection'
      variable_param.prob_connection = k;
    case 'prob_rel_measurement'
      variable_param.prob_rel_measurement = k;
    otherwise 
      error('Case not implemented yet!')
  end

  par = parameters(variable_param, user_par,par);

  % Set to 1 the parameters not investigated
  switch param
    case 'prob_GPS'
      par.prob_connection = 1;
      par.prob_rel_measurement = 1;
    case 'prob_connection'
      par.prob_GPS = 1;
      par.prob_rel_measurement = 1;
    case 'prob_rel_measurement'
      par.prob_GPS = 1;
      par.prob_connection = 1;
    otherwise 
      error('Case not implemented yet!')
  end

end