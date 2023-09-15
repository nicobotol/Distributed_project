function user_par = get_user_input()
    
  disp('<strong>-------------------------------------------------------------------------</strong>')
  disp('<strong>Beginning of simulation</strong>')
  disp('-------------------------------------------------------------------------')

  % Choose the number of parachutes
  prompt1 = "Number of parachutes: ";
  user_par.n_agents = input(prompt1);
  if isempty(user_par.n_agents)
    disp('No input inserted, the simulation will start with 1 parachute.')
    user_par.n_agents = 1;
  end
  
  % Choose the initial position
  prompt2 = "Choose initial x position: ";
  x = input(prompt2);
  if isempty(x)
    disp('No input inserted, the simulation will start with initial x = 50.')
    x = 50;
  end
  prompt3 = "Choose initial y position: ";
  y = input(prompt3);
  if isempty(y)
    disp('No input inserted, the simulation will start with initial y = 50.')  
    y = 50;
  end
  prompt4 = "Choose initial z position: ";
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

  disp('-------------------------------------------------------------------------')
  
  end