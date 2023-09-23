function [chute, post_process_data, true_centroid_store, par, w_store] = simulation(par, k, post_process_data)
%% This function runs the basic cycle performing the simualtion

  %% Initialization
  [chute, ground_check, true_centroid_store, w_store] = initialization_chutes(par);

  if par.enable_video == 1
    v = VideoWriter('chutes.mp4','MPEG-4'); % name of the video file
    v.FrameRate = 10; % set the desired number of frames per second
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
    chute = distribute_positions_WLS(chute, par, t);
       
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
    [j_fig, chute] = plot_chutes_time_evo(chute, true_centroid_store, t, par);
    if par.enable_video == 1
      frame = getframe(gcf);
      writeVideo(v,frame);
    end
    
  end

  if par.enable_video == 1
    close(v);
  end

  tot_time = toc(t_start);
  fprintf("The simulation ended in %.2f [s] = %.2f [min]\n", tot_time, tot_time/60);
  pp_tmp = post_process_data;
  post_process_data = post_process(chute, k, post_process_data, par);
  
end