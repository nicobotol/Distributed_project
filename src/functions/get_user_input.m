function [par, parametric] = get_user_input(par)

% Choose the number of parachutes
prompt1 = "Number of parachutes: ";
par.n_agents = input(prompt1);
if isempty(par.n_agents)
  par.n_agents = 1;
end

% Choose the initial position
prompt2 = "Choose initial x position: ";
par.x0 = input(prompt2);
if isempty(par.x0)
  par.x0(1) = 50;
end
prompt3 = "Choose initial y position: ";
par.y0 = input(prompt3);
if isempty(par.y0)
  par.x0(2) = 50;
end
prompt4 = "Choose initial z position: ";
par.x0(3) = input(prompt4);
if isempty(par.x0(3))
  par.x0(3) = 500;
end

% Choose whether to do a parametric analysis
prompt4 = "Parametric analysis? (1 for yes, 0 for no): ";
parametric = input(prompt4);
if isempty(parametric)
  parametric = 0;
end

% Choose the model
prompt5 = "Choose the model (1 for linear (drone), 2 for non-linear (parachute)): ";
par.mdl = input(prompt5);
if isempty(par.mdl)
  par.mdl = 1;
end

end