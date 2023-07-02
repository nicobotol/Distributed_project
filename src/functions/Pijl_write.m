function agents = Pijl_write(agents, Pijl, i, j, l)

  if l>j
    agents{i}.P{j, l} = Pijl;
  else
    agents{i}.P{l, j} = Pijl;
  end
end