function user_par = get_user_input(parametric)
    
  disp('<strong>-------------------------------------------------------------------------</strong>')
  disp('<strong>Beginning of simulation</strong>')
  disp('-------------------------------------------------------------------------')

  if parametric == 0
    % Choose the number of parachutes
    prompt1 = "Number of parachutes: ";
    user_par.n_agents = input(prompt1);
    if isempty(user_par.n_agents)
      disp('No input inserted, the simulation will start with 1 parachute.')
      user_par.n_agents = 1;
    end
  elseif parametric == 1
    disp('A parametric simulation will be performed.')
    prompt1 = "Number of parachutes: ";
    user_par.n_agents = input(prompt1);
    if isempty(user_par.n_agents)
      disp('No input inserted, the simulation will start with 1 parachute.')
      user_par.n_agents = 1;
    end
  elseif parametric == 2
    % Choose the number of seeds
    disp('An IK simulation will be performed.')
    prompt1 = "Number of seeds: ";
    user_par.n_simulation = input(prompt1);
    if isempty(user_par.n_simulation)
      disp('No input inserted, the simulation will start with 2 seeds.')
      user_par.n_simulation = 2;
    end
    user_par.n_agents = 1;
  else
    error('Wrong input for parametric')
  end
  
  % Choose the initial position
  prompt2 = "Initial x position: ";
  x = input(prompt2);
  if isempty(x)
    disp('No input inserted, the simulation will start with initial x = 50.')
    x = 50;
  end
  prompt3 = "Initial y position: ";
  y = input(prompt3);
  if isempty(y)
    disp('No input inserted, the simulation will start with initial y = 50.')  
    y = 50;
  end
  prompt4 = "Initial z position: ";
  z = input(prompt4);
  if isempty(z)
    disp('No input inserted, the simulation will start with initial z = 500.')  
    z = 500;
  end

  user_par.x0 = [x, y, z]';
  
  % Choose the model
  prompt5 = "Choose the model (1 for linear (drone), 2 for non-linear (parachute)): ";
  user_par.mdl = input(prompt5);
  if isempty(user_par.mdl)
    disp('No input inserted, the simulation will start with model 1.')  
    user_par.mdl = 1;
  end

  % Choose if IK or not
  if parametric ~= 2
    prompt6 = "Choose if using IK (1 for IK, 0 otherwise): ";
    user_par.IK = input(prompt6);
    if isempty(user_par.IK)
      disp('No input inserted, the simulation will start with IK.')  
      user_par.IK = 1;
    end
  else
    user_par.IK = 1;
  end

  disp('-------------------------------------------------------------------------')
  
  end