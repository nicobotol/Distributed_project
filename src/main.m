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
[chute, ground_check, true_centroid_store] = initialization_chutes();
  
t = 0;
while (t < T && prod(ground_check) < 1)
  t = t + 1; 
  % %% Localization and measurement
  % chute = localization_chutes(chute);

  % %% Distribute informations
  % chute = distribute_informations(chute);

  chute = localization_chutes_DKF(chute);

  %% Compute the global centroid
  chute = wls_centroid(chute);

  %% Voronoi
  chute = voronoi_chutes(chute);

  %% Plot
  [j_fig] = plot_chutes_time_evo(chute, true_centroid_store, t);

  %% Dynamic
  [chute, ground_check, true_centroid_store] = dynamic_chutes(chute, t, ground_check, true_centroid_store);

end

%% Plot
plot_chutes_trajectory(chute, true_centroid_store, j_fig);