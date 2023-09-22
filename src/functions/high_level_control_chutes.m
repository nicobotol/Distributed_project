function [agents, w_store] = high_level_control_chutes(agents, t, w_store, par)
% This function computes the high level control: it moves the global centroid and then assign the target point ot each of the agents

n_agents = par.n_agents;
dt = par.dt;
IK = par.IK;
A = par.A;
B = par.B;
S = par.S;
R = par.R;
target = par.target;
centroid_states_len = par.centroid_states_len;

for i=1:n_agents
    if agents{i}.terminal_speed == 1
      if isempty(agents{i}.near_z)
        % LQR gain matrix
        K = dlqr(A(1:2,1:2), B(1:2,1:2), S, R);
        
        % LQR input: input that the i-th agent would apply at its own centroid, but in turns it applies to itself (i.e. the agents apply to itself the inputs that applies to the centroid)
        u_global_centroid = -K*(agents{i}.global_centroid(1:2) - target(1:2));   
      
        % Choose wheter to use the inverse kinematics or not
        if IK == 1
          [sim_x, w] = inverse_kinematics(agents{i}, i, u_global_centroid, dt, K, centroid_states_len - 1, A, B, t, par);
        else
          u_sim = u_global_centroid;
          sim_x = A(1:2,1:2)*agents{i}.x(1:2, i) + B(1:2,1:2)*u_sim;
        end
        % Simulate the new chute position (i.e. the position where the chute has to go to fulfill the movement of the centroid as desired by the LQR)
        if IK == 1 && i == 1
          w_store = [w_store, w];
        end
      else % case of chute spown near by us
        vector = zeros(2, size(agents{i}.near_z, 2));
        for j=1:size(agents{i}.near_z, 2)
          vector(:, j) = agents{i}.x(1:2, agents{i}.near_z(j)) - agents{i}.x(1:2, i); % vector from the surrounding points to the agent i
        end
        sim_x = -100*sum(vector, 2); % sum vector
      end
      
      % Extract final agents positions: in order to avoid that the point is placed too far, then limit the maximum distance between the current location and the target point to a reasonably high value (keeping the same orientation)
      dist = norm(sim_x(:, 1) - agents{i}.x(1:2, i)); % dist location-target
      dist_th = 1000; % threshold above which move the target closer
      if dist < dist_th
      else
        dir = (sim_x(:, 1) - agents{i}.x(1:2, i))/dist; % direction
        sim_x = agents{i}.x(1:2, i) + dist_th*dir;
      end
      
      agents{i}.sim_x = sim_x;


    else
        agents{i}.vz = agents{i}.nu(4);
    end
end