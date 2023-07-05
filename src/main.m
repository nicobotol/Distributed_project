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

  %% Dynamic
  % if mdl == 2 
  %   [chute, ground_check, true_centroid_store] = dynamic_chutes_m2(chute, ground_check, true_centroid_store, t);
  % elseif mdl == 4
  %   [chute, ground_check, true_centroid_store] = dynamic_chutes_m4(chute, ground_check, true_centroid_store, t);
  % elseif mdl == 6
  %   [chute, ground_check, true_centroid_store] = dynamic_chutes_m6(chute, ground_check, true_centroid_store, t);
  % end

  %% Generate the disturbances on the input
  % chute = generate_disturbance(chute);

  % chute = localization_chutes_DKF(chute);  % DKF with relative mes., P not diagonal
  % chute = localization_chutes_DKF2(chute);  % DKF with relative mes. (1 agent at time)
  % chute = localization_chutes_DKF3(chute);  % DKF without relative mes., P diagonal
  



  %% Plot
%   [j_fig, chute] = plot_chutes_time_evo(chute, true_centroid_store, t);
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