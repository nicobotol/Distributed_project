function impact_detection_chutes(agents, true_centroid_store, t, par)
  %% This function detects whether two agents had collided each other

  n_agents = par.n_agents;
  if t ~= 1
    for i = 1:n_agents
      for j = i+1:n_agents
        if i~=j
          dist = norm(agents{i}.x_real(1:2) - agents{j}.x_real(1:2));
          dist_z = agents{i}.x_real(3) - agents{j}.x_real(3);
          sign_z = dist_z/abs(dist_z);

          sum_delta = agents{i}.delta + agents{j}.delta;

          if dist < sum_delta && ((dist_z < agents{j}.z_th && sign_z > 0) || (-dist_z < agents{i}.z_th && sign_z < 0))
            fprintf('Collision between agents %d and %d at step %d\n', i, j, t);

            dist_z_prev = agents{i}.x_real_store(3, t-1) - agents{j}.x_real_store(3, t-1);
          end
        end
      end
    end
  end
end
