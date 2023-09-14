function [user_par, parametric] = get_user_input()

% Choose the number of parachutes
prompt1 = "Number of parachutes: ";
user_par.n_agents = input(prompt1);
if isempty(user_par.n_agents)
  user_par.n_agents = 1;
end

% Choose the initial position
prompt2 = "Choose initial x position: ";
user_par.x0(1) = input(prompt2);
if isempty(user_par.x0(1))
  user_par.x0(1) = 50;
end
prompt3 = "Choose initial y position: ";
user_par.x0(2) = input(prompt3);
if isempty(user_par.x0(2))
  user_par.x0(2) = 50;
end
prompt4 = "Choose initial z position: ";
user_par.x0(3) = input(prompt4);
if isempty(user_par.x0(3))
  user_par.x0(3) = 500;
end

% Choose whether to do a parametric analysis
prompt4 = "Parametric analysis? (1 for yes, 0 for no): ";
parametric = input(prompt4);
if isempty(parametric)
  parametric = 0;
end

% Choose the model
prompt5 = "Choose the model (1 for linear (drone), 2 for non-linear (parachute)): ";
user_par.mdl = input(prompt5);
if isempty(user_par.mdl)
  user_par.mdl = 1;
end

end