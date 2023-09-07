function agents = generate_disturbance(agents)
%% This function generates the disturbance on the not controllabel inputs

parameters;

if mdl == 2

elseif mdl == 4
  for i = 1:n_agents
    % external disturbance
    agents{i}.nu(:) = nu_mag*randn(nc_inputs_len,1);
    agents{i}.nu(1) = V;     
    agents{i}.nu(5) = -V_z;
    agents{i}.nu_unc(:) = agents{i}.nu(:) + mvnrnd([0;0;0;0;0;0], agents{i}.L)'; 
  end

else
  error('Model not implemented')
end

end