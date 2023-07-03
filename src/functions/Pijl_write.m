function agents = Pijl_write(agents, Pijl, i, j, l)

  if l>j
    agents{i}.Pi{j, l} = Pijl;
  else
    agents{i}.Pi{l, j} = Pijl';
  end
end