function [result, chute] = comparison_IK(user_par, variable_param, par)

k = 1;
ag_number = par.ag_number;
ag = par.ag;
parametric = par.parametric;
for i=1:user_par.n_simulation % loop over different seed
  user_par.seed = i;
  par = parameters(variable_param, user_par, par);    
  par = remove_noise(par);
  par.parametric = parametric;
  for j=1:2 % loop bewteen IK or noIK
    if j==1
      par.IK = 1;
    else
      par.IK = 0;
    end
    
    for l=1:ag_number
      par.n_agents = ag(l);
      post_process_data = cell(1,1);
      [chute, post_process_data, true_centroid_store, par, w_store] = simulation(par, k, post_process_data);
      result{l}.agents = par.n_agents; 
      if j==1
        [result{l}.IK.RMS(i), result{l}.IK.dist(i)] = RMS_final_chute(chute, par, true_centroid_store);
      else
        [result{l}.no_IK.RMS(i), result{l}.no_IK.dist(i)] = RMS_final_chute(chute, par, true_centroid_store);
      end
    end

  end
end

% Do the mean of the values inside the structure
for i=1:ag_number
  result{i}.IK.RMS_mean = mean(result{i}.IK.RMS);
  result{i}.IK.dist_mean = mean(result{i}.IK.dist);
  result{i}.no_IK.RMS_mean = mean(result{i}.no_IK.RMS);
  result{i}.no_IK.dist_mean = mean(result{i}.no_IK.dist);
end

name = ['code_for_report/IK_mod', num2str(par.mdl), '.tex'];
fid = fopen(name,'w');
fprintf(fid, '\\begin{table}[]\n');
fprintf(fid, '\\caption{Comparison between the use of IK or not}\n');
fprintf(fid, '\\centering\n');
fprintf(fid, '\\begin{tabular}{ccccc}\n');
fprintf(fid, '\\hline\n');
fprintf(fid, '\\multicolumn{1}{c}{\\# chutes} & \\multicolumn{2}{c}{Dist.} & \\multicolumn{2}{c}{RMS} \\\\ \n');
fprintf(fid, '\\multicolumn{1}{l}{}  & IK & No IK & IK & No IK \\\\ \\hline\n');
for i=1:ag_number
  fprintf(fid,'%d & %d & %.2f & %.2f & %.2f & %.2f \\\\\n',i,result{i}.agents,result{i}.IK.dist_mean,result{i}.no_IK.dist_mean,result{i}.IK.RMS_mean,result{i}.no_IK.RMS_mean);
end
fprintf(fid, '\\hline\n');
fprintf(fid, '\\end{tabular}\n');
fprintf(fid, '\\end{table}\n');
fclose(fid);


end