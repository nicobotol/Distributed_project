% clc
% close all
clear

if ispc
  path = 'functions\';
else
  path = 'functions/';
end

addpath(path);
initialize_environment;

%% Load parameters
parameters;

%% Initialization
[chute, ground_check, true_centroid_store, w_store] = initialization_chutes();

if enable_video == 1
  v = VideoWriter('chutes.mp4','MPEG-4');
  v.FrameRate = 1;
  open(v);
end

t = 0;

while (t < T && prod(ground_check) < 1)
  t = t + 1; 
  %% Generate the external disturbance
  chute = external_disturbance_chutes(chute, t);

  %% Dynamic
  [chute, ground_check] = dynamic_chutes(chute, ground_check, t);

  %% Localization of the chutes via KF (each agents uses its own KF)
  chute = localization_chutes_KF(chute, ground_check); 

  %% Distribute the positions via WLS
  chute = distribute_positions_WLS(chute);

  %% Compute the local estimation of the global centroid 
  chute = wls_global_centroid(chute);

  %% Store the estimation
  chute = store_control_chutes(chute, ground_check);

  %% Perform the Voronoi tesslation
  chute = voronoi_chutes(chute);

  %% Plot the time evolution
  % [j_fig, chute] = plot_chutes_time_evo(chute, true_centroid_store, t);

  %% High level control
  [chute, w_store] = high_level_control_chutes(chute, t, w_store);

  %% Centroid of the voronoi cell
  tic
  chute = voronoi_cell_centroid(chute, t); 
toc
  %% Store the global centroid into a variable
  [chute, true_centroid_store] = global_centroid_chutes(chute, true_centroid_store, t);

  %% Compute the low level control
  chute = low_level_control_chute(chute, t); 

  %% Detect impacts
  impact_detection_chutes(chute, true_centroid_store, t);

  if enable_video == 1
    frame = getframe(gcf);
    writeVideo(v,frame);
  end
  
end

if enable_video == 1
  close(v);
end

%% Plot
if(exist("j_fig") == 0)
  j_fig = 0;
end
plot_chutes_trajectory(chute, true_centroid_store, j_fig, w_store);

RMS_final_chute(chute);