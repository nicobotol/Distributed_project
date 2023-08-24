function [agents, w_store] = high_level_control_chutes(agents, t, w_store)
% This function computes the high level control: it moves the global centroid and then assign the target point ot each of the agents

parameters;


for i=1:n_agents
    if agents{i}.terminal_speed == 1
      % LQR gain matrix
      % K = lqr_finite(A, B(1:3,1:2), S, R, T, Sf, states_len, inputs_len-1, mdl, t);
      K = dlqr(A(1:2,1:2), B(1:2,1:2), S, R);
      
      % LQR input: input that the i-th agent would apply at its own centroid, but in turns it applies to itself (i.e. the agents apply to itself the inputs that applies to the centroid)
      u_global_centroid = -K*(agents{i}.global_centroid(1:2) - target(1:2));   
    
      % Choose wheter to use the inverse kinematics or not
      if IK == 1
        [sim_x, w] = inverse_kinematics(agents{i}, i, u_global_centroid, dt, K, centroid_states_len - 1, A, B, t);
      else
        u_sim = u_global_centroid;
        sim_x = A(1:2,1:2)*agents{i}.x(1:2, i) + B(1:2,1:2)*u_sim;
      end
      % Simulate the new chute position (i.e. the position where the chute has to go to fulfill the movement of the centroid as desired by the LQR)
      agents{i}.sim_x = sim_x;

      if IK == 1
        w_store = [w_store, w];
      end

    else
        agents{i}.vz = agents{i}.nu(4);
    end
end