%% Share in each agent the information about the position of the others
function agents = distribute_informations(agents)

n_agents = length(agents);

for i = 1:n_agents
  for j = 1:n_agents
    if i ~= j
      dist3D = norm(agents{i}.x - agents{j}.x); % distance between robots in 3D space
      % add a robot in the neightbours set only if it is inside in the communication range and if it is at the seme height
      if dist3D <= agents{i}.Rc
        
        agents{i}.agents_position = [agents{i}.agents_position agents{j}.x]; % position of the agents j known by i
        
      end
    end
  end
end

% Compute the global centroid (i.e. centroid of the storm)
for i=1:n_agents
  agents{i}.global_centroid = mean([agents{i}.x agents{i}.agents_position],2);
end

end