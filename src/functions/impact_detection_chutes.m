function impact_detection_chutes(agents, true_centroid_store, t)
  %% This function detects whether two agents had collided each other

  parameters
  for i = 1:n_agents
    for j = i+1:n_agents
      if i~=j
        dist = norm(agents{i}.x_real(1:2) - agents{j}.x_real(1:2));
        dist_z = agents{i}.x_real(3) - agents{j}.x_real(3);
        sign_z = dist_z/abs(dist_z);

        sum_delta = agents{i}.delta + agents{j}.delta;

        if dist < sum_delta && ((dist_z < agents{j}.z_th && sign_z > 0) || (-dist_z < agents{i}.z_th && sign_z < 0))
          fprintf('Collision between agents %d and %d at step %d\n', i, j, t);
          % plot_chutes_time_evo(agents, true_centroid_store, t);
        end
      end
    end
  end
