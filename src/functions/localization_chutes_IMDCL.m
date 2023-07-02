function [agents] = localization_chutes_IMDCL(agents)
  % implement the Interim Master Decentralized Cooperative Localization algorithm for the chute localization

parameters; % load parameters
n_agents = length(agents); % number of agents

% Propagation
for i=1:n_agents
  agents{i}.x_hat_m = A*agents{i}.x_hat_p + B*agents{i}.u + G(3, 4)*agents{i}.nu(4);
  agents{i}.P_m = A*agents{i}.P_p*A' + B*agents{i}.Q*B';
  agents{i}.Phi = A*agents{i}.Phi;
end

rel_mes_network = zeros(n_agents, n_agents); % matrix to see if there are relative measurement in the network
for i = 1:n_agents
  for j = 1:n_agents
    if i ~= j
      dist = norm(agents{i}.x_real - agents{j}.x_real); % distance between agents
      if dist <= agents{i}.Rc
        rel_mes_network(i, j) = 1; % mark if there is a realtive measurement between i and j
      end
    end
  end
end

if max(rel_mes_network, [], 'all') == 0 % no relative measuremnts in the network
  for i=1:n_agents
    agents{i}.x_hat_p = agents{i}.x_hat_m;
    agents{i}.P_p = agents{i}.P_m;
    for j = 1:max(n_agents)-1
      for l = j+1:max(n_agents)
        agents{i}.Pi{j,l} = agents{i}.Pi{j,l};
      end
    end
  end
else % there is a relative measurements in the network
  for a = 1:n_agents
    for b = 1:n_agents
       if rel_mes_network(a, b) == 1
        % i acquire informations from j
        lm_x_hat_bm = agents{b}.x_hat_m; % landmark message
        lm_Phi_b = agents{b}.Phi; % landmark message
        lm_P_bm = agents{b}.P_m; % landmark message

        % calculation done by i upone receving the landmark message
        % r^a
        zab = (agents{a}.x_real(1:measure_len) - agents{b}.x_real(1:measure_len)) + mvnrnd(zeros(measure_len, 1), agents{a}.R_relative)'; % relative measurement from i to j
        hab = agents{a}.x_hat_m - agents{b}.x_hat_m; % measurement model applied to 
        ra = zab - hab; % innovation of the relative measurement
        % S_ab
        H_tilde_a = agents{a}.H_tilde;
        H_tilde_b = agents{b}.H_tilde;
        Phi_a = agents{a}.Phi;
        [Pa_ab, Pa_ba] = Pijl_read(agents, a, a, b);
        Sab = agents{a}.R_relative + H_tilde_a*agents{a}.P_m*H_tilde_a' + H_tilde_b*agents{b}.P_m*H_tilde_b' - H_tilde_a*Phi_a*Pa_ab*lm_Phi_b'*H_tilde_b' - H_tilde_b*lm_Phi_b*Pa_ba*Phi_a'*H_tilde_a';   
        % Gamma_a and Gamma_b
        agents{a}.Gamma{a} = (inv(Phi_a)*Phi_a*Pa_ab*agents{b}.Phi'*H_tilde_b - inv(Phi_a)*agents{a}.P_m*H_tilde_a')*Sab^(-1/2); 
        agents{a}.Gamma{b} = (inv(lm_Phi_b)*lm_P_bm*H_tilde_b' - Pa_ba*Phi_a'*H_tilde_a')*Sab^(-1/2);

        % update messages
        msg1 = lm_Phi_b*H_tilde_b*Sab^(-1/2);
        msg2 = Phi_a*H_tilde_a'*Sab^(-1/2);

        for i=1:n_agents
          for j=1:n_agents
            if j~=a && j~=b
              [Pi_jb, ~] = Pijl_read(agents, i, j, b);
              [Pi_ja, ~] = Pijl_read(agents, i, j, a);

              agents{i}.Gamma{j} = Pi_jb*msg1 - Pi_ja*msg2;

            end
          end
          
          ra_bar = Sab^(-1/2)*ra;
          agents{i}.x_hat_p = agents{i}.x_hat_m + agents{i}.Phi*agents{i}.Gamma{i}*ra_bar;
          agents{i}.P_p = agents{i}.P_m - agents{i}.Phi*agents{i}.Gamma{i}*agents{i}.Gamma{i}'*agents{i}.Phi';
          for j = 1:max(n_agents) - 1 % j
            for l = j+1:max(n_agents) % l
              [Pi_jl, ~] = Pijl_read(agents, i, j, l);
              Pi_jl = Pi_jl - agents{i}.Gamma{j}*agents{i}.Gamma{l}';
              agents = Pijl_write(agents, Pi_jl, i, j, l);
            end
          end

        end
      end
    end 
  end
end

for i = 1:n_agents
  for j = 1:n_agents
    if i == j
      agents{i}.P_est{j} = agents{i}.P_p;
    else
      [Pi_ij, ~] = Pijl_read(agents, i, i, j);
      agents{i}.P_est{j} = agents{i}.Phi*Pi_ij*agents{j}.Phi';
    end
    agents{i}.x(:, j) = agents{j}.x_hat_p;
  end
end

% %% OLD FORMULATION
% % Update
% for i=1:n_agents
%   for j = 1:n_agents
%     if i ~= j
%       dist = norm(agents{i}.x_real(1:2) - agents{j}.x_real(1:2)); % distance between agents

%       if dist > agents{i}.Rc % no relative measurements
%         agents{i}.x_hat_p = agents{i}.x_hat_m;
%         agents{i}.P_p = agents{i}.P_m;
%         for j = 1:max(n_agents)-1
%           for l = j+1:max(n_agents)
%             agents{i}.Pi{j,l} = agents{i}.Pi{j,l};
%           end
%         end
%       else  % relative measurements between i and j
%         % i acquire informations from j
%         lm_x_hat_bm = agents{j}.x_hat_m; % landmark message
%         lm_Phi_b = agents{j}.Phi; % landmark message
%         lm_P_bm = agents{j}.P_m; % landmark message

%         % calculation done by i upone receving the landmark message
%         % r^a
%         zij = (agents{j}.x_real(1:measure_len) - agents{i}.x_real(1:measure_len)) + mvnrnd(zeros(measure_len, 1), agents{i}.R_relative)'; % relative measurement from i to j
%         hij = agents{i}.x_hat_m - agents{j}.x_hat_m; % measurement model applied to 
%         rij = zij - hij; % innovation of the relative measurement
%         % S_ab
%         H_tilde_i = agents{i}.H_tilde;
%         H_tilde_j = agents{j}.H_tilde;
%         Phi_i = agents{i}.Phi;
%         Sij = agents{i}.R_relative + H_tilde_i*agents{i}.P_m*H_tilde_i' + H_tilde_j*agents{j}.P_m*H_tilde_j' - H_tilde_i*Phi_i*agents{i}.Pi{i, j}*lm_Phi_b'*H_tilde_j' - H_tilde_j*lm_Phi_b*agents{i}.Pi{j, i}*Phi_i'*H_tilde_i';

%         % Gamma_a and Gamma_b
%         Gamma_i = (inv(Phi_i)*Phi_i*agents{i}.Pi{i, j}*agents{j}.Phi'*H_tilde_j - inv(Phi_i)*agents{i}.P_m*H_tilde_i')*Sij^(-1/2); 
%         Gamma_j = (inv(lm_Phi_b)*lm_P_bm*H_tilde_j' - agents{i}.Pi{j, i}*Phi_i'*H_tilde_i')*Sij^(-1/2);
  
%         % update the other agents
%         for k=1:n_agents % agents to which the master sends informations
%           for l=1:n_agents
%             if l ~= i && l ~= k
%               agents{k}.Gamma{j} = agents{k}.P{l, j}*Gamma_j'*H_tilde_j*Sij^(-1/2) - agents{i}.P{j, i}*Gamma_i'*H_tilded_i'*Sij^(-1/2);
%             end
%           end
%           agents{k}.x_hat_p = agents{k}.x_hat_m + agents{k}.Phi*agents{k}.Gamma{l}*rij;
%           agents{k}.P_p = agents{k}.P_m - agents{k}.Phi*agents{k}.Gamma{k}*agents{k}.Gamma{k}'*agents{k}.Phi';
%           for ll = 1:max(n_agents) - 1 % j
%             for lll = ll+1:max(n_agents) % l
%               agents{k}.Pi{ll, lll} = agents{k}.Pi{ll, lll} - agents{k}.Gamma{ll}*agents{k}.Gamma{lll}';
%             end
%           end
%         end
%       end 
%     end
%   end
% end
  
