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
  
  %% Generate the disturbances on the input
  chute = generate_disturbance(chute);

  %% Localization and consensus on the positions
  chute = localization_chutes_KF_WLS(chute); % single KF + WLS
  % chute = localization_chutes_DKF(chute);  % DKF with relative mes., P not diagonal
  % chute = localization_chutes_DKF2(chute);  % DKF with relative mes. (1 agent at time)
  % chute = localization_chutes_DKF3(chute);  % DKF without relative mes., P diagonal
  
  %% Compute the global centroid
  chute = wls_centroid(chute);

  %% Voronoi
  chute = voronoi_chutes(chute);

  %% Plot
  [j_fig] = plot_chutes_time_evo(chute, true_centroid_store, t);
  
  %% Dynamic
  if mdl == 2
    [chute, ground_check, true_centroid_store] = dynamic_chutes_m2(chute, t, ground_check, true_centroid_store, t);
  elseif mdl == 4
    [chute, ground_check, true_centroid_store] = dynamic_chutes_m4(chute, t, ground_check, true_centroid_store, t);
  end
end

%% Plot
plot_chutes_trajectory(chute, true_centroid_store, j_fig);