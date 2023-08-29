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
number_simulations = 2; % write here the number of simulations to be performed
variable_param.par1 = 0;
variable_param.par2 = 0;

for k=1:number_simulations
  %% Set iterator for the different parameters to be tested
  variable_param.par1 = variable_param.par1 + 1;
  variable_param.par2 = variable_param.par2 + 1;
  
  %% Load parameters
  par = parameters(variable_param);

  %% Initialization
  [chute, ground_check, true_centroid_store, w_store] = initialization_chutes(par);

  if par.enable_video == 1
    v = VideoWriter('chutes.mp4','MPEG-4');
    v.FrameRate = 1;
    open(v);
  end

  t = 0;
  t_start = tic;
  while (t < par.T && prod(ground_check) < 1)
    t = t + 1;

    %% Generate the external disturbance
    chute = external_disturbance_chutes(chute, t, par);
    
    %% Dynamic
    [chute, ground_check] = dynamic_chutes(chute, ground_check, par.dt, par);
    
    %% Free Falling
    chute = free_falling(chute, t, par);

    %% Localization of the chutes via KF (each agents uses its own KF)
    chute = localization_chutes_KF(chute, ground_check, t, par); 
       
    %% Distribute the positions via WLS
    chute = distribute_positions_WLS(chute, par);
       
    %% Compute the local estimation of the global centroid 
    chute = wls_global_centroid(chute, par);
        %% Store the estimation
    chute = store_control_chutes(chute, ground_check, par);
       
    %% Perform the Voronoi tesslation
    chute = voronoi_chutes(chute, t, par);
       
    %% High level control
    [chute, w_store] = high_level_control_chutes(chute, t, w_store, par);
       
    %% Centroid of the voronoi cell
    chute = voronoi_cell_centroid(chute, t, par); 
       
    %% Store the global centroid into a variable
    [chute, true_centroid_store] = global_centroid_chutes(chute, true_centroid_store, t, par);
        
    %% Compute the low level control
    chute = low_level_control_chute(chute, t, par); 
        
    %% Detect impacts
    impact_detection_chutes(chute, true_centroid_store, t, par);
    
    %% Plot the time evolution
    % [j_fig, chute] = plot_chutes_time_evo(chute, true_centroid_store, t, par);
    if par.enable_video == 1
      frame = getframe(gcf);
      writeVideo(v,frame);
    end
      
  end
  
  tot_time = toc(t_start);
  fprintf("The simulation ended in %.2f [s] = %.2f [min]\n", tot_time, tot_time/60);
end
  
if par.enable_video == 1
  close(v);
end

%% Plot
if(exist("j_fig") == 0)
  j_fig = 0;
end
plot_chutes_trajectory(chute, true_centroid_store, j_fig, w_store, par);

RMS_final_chute(chute, par);