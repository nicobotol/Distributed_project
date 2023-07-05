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

if enable_video == 1
  v = VideoWriter('chutes.mp4','MPEG-4');
  v.FrameRate = 1;
  open(v);
end

t = 0;

while (t < T && prod(ground_check) < 1)
  t = t + 1; 
  %% Dynamic
  [chute, ground_check] = dynamic_chutes(chute, ground_check, t);
  %% Localization and consensus on the positions
  chute = localization_chutes_KF_WLS(chute, ground_check); % single KF + WLS
  %% Distribute the positions
  chute = distribute_informations2(chute);
  %% Voronoi
  chute = voronoi_chutes(chute);
%   [j_fig, chute] = plot_chutes_time_evo(chute, true_centroid_store, t);
  %% centroid of the voronoi cell
  chute = voronoi_centroid(chute, t); 
  %% Compute the global centroid 
  chute = wls_centroid(chute);
  %% Store the global centroid into a variable
  [chute, true_centroid_store] = global_centroid_chutes(chute, true_centroid_store, t);
  %% compute the low level control
  chute = low_level_control_chute(chute, t); 

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
plot_chutes_trajectory(chute, true_centroid_store, j_fig);