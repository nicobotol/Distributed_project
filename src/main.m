clc
close all
clear

% Choose slash according to your operating system automatically
if ispc
    path = 'functions\';
else
    path = 'functions/';
end

addpath(path);

%% Load parameters
parameters;

%% Initialization
chute = initialization_chutes();

t = 0;
while (t < T && chute{1}.global_centroid(3) > 0)
  t = t + 1; 
  %% Localization and measurement
  chute = localization_chutes(chute);

  %% Distribute informations
  chute = distribute_informations(chute);

  %% Voronoi
  chute = voronoi_chutes(chute);

  %% Dynamic
  chute = dynamic_chutes(chute, t);

  %% Plot
  plot_chutes_time_evo(chute)

end

%% Plot
plot_chutes_trajectory(chute);