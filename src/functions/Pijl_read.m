function [Pi_jl, Pi_lj] = Pijl_read(agents, i, j, l)
  % check if it is necessary to revert the agents{i}.Pi_{ij}

  if ~isempty(agents{i}.Pi{j, l})
    Pi_jl = agents{i}.Pi{j, l};
    Pi_lj = (agents{i}.Pi{j, l})';
  else
    Pi_jl = (agents{i}.Pi{l, j})';
    Pi_lj = agents{i}.Pi{j, l};
  end