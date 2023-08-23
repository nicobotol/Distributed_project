clc
close all
clear

format short

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
t_start = tic;
while (t < T && prod(ground_check) < 1)
  t = t + 1; 
  %% Generate the external disturbance
  tic
  chute = external_disturbance_chutes(chute, t);
  stopwatch(1, t) = toc;
  
  %% Dynamic
  tic
  [chute, ground_check] = dynamic_chutes(chute, ground_check, dt);
  stopwatch(2, t) = toc;

  %% Localization of the chutes via KF (each agents uses its own KF)
  tic
  chute = localization_chutes_KF(chute, ground_check, t); 
  stopwatch(3, t) = toc;
  
  %% Distribute the positions via WLS
  tic
  chute = distribute_positions_WLS(chute);
  stopwatch(4, t) = toc;
  
  %% Compute the local estimation of the global centroid 
  tic
  chute = wls_global_centroid(chute);
  stopwatch(5, t) = toc;

  %% Store the estimation
  tic
  chute = store_control_chutes(chute, ground_check);
  stopwatch(6, t) = toc;
  
  %% Perform the Voronoi tesslation
  tic
  chute = voronoi_chutes(chute, t);
  stopwatch(7, t) = toc;
  
  %% High level control
  tic
  [chute, w_store] = high_level_control_chutes(chute, t, w_store);
  stopwatch(8, t) = toc;
  
  %% Centroid of the voronoi cell
  tic
  chute = voronoi_cell_centroid(chute, t); 
  stopwatch(9, t) = toc;
  
  %% Store the global centroid into a variable
  tic
  [chute, true_centroid_store] = global_centroid_chutes(chute, true_centroid_store, t);
  stopwatch(10, t) = toc;
  
  %% Compute the low level control
  tic
  chute = low_level_control_chute(chute, t); 
  stopwatch(11, t) = toc;
  
  %% Detect impacts
  tic
  impact_detection_chutes(chute, true_centroid_store, t);
  stopwatch(12, t) = toc;

  %% Plot the time evolution
  [j_fig, chute] = plot_chutes_time_evo(chute, true_centroid_store, t);
  if enable_video == 1
    frame = getframe(gcf);
    writeVideo(v,frame);
  end
  
end
tot_time = toc(t_start);

if enable_video == 1
  close(v);
end

%% Plot
if(exist("j_fig") == 0)
  j_fig = 0;
end
plot_chutes_trajectory(chute, true_centroid_store, j_fig, w_store);

RMS_final_chute(chute);