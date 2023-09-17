% Values are computed three times each of them with different seed for the random number generator (in the order they are 6, 8, 4, 2).
% The reults are computed with or without the use of inverse kinematic IK, with a different numbe of agents, starting from the posizion of x0=[500,500,1000] and the model 2 (unicycle)


test{1}.agents = 1;
test{1}.IK.dist = [3.85; 5.56; 2.83];
test{1}.IK.RMS = [3.85; 5.56; 2.83];
test{1}.no_IK.dist = [5.74; 1.86; 3.64]; 
test{1}.no_IK.RMS = [5.74; 1.86; 3.64]; 

test{2}.agents = 3;
test{2}.IK.dist = [8.18; 10.57; 5.21];
test{2}.IK.RMS = [27.85; 31.79; 29.06];
test{2}.no_IK.dist = [2.04; 10.21; 8.44];
test{2}.no_IK.RMS = [31.68; 28.90; 31.06];

test{3}.agents = 5;
test{3}.IK.dist = [6.66; 14.19; 10.87];
test{3}.IK.RMS = [40.32; 46.78; 44.14];
test{3}.no_IK.dist = [4.82; 4.25; 9.84];
test{3}.no_IK.RMS = [45.56; 42.73; 44.92];

test{4}.agents = 7;
test{4}.IK.dist = [14.54; 13.14; 6.89];
test{4}.IK.RMS = [44.71; 49.55; 47.41];
test{4}.no_IK.dist = [7.32; 17.58; 6.11];
test{4}.no_IK.RMS = [47.26; 52.42; 50.40];

test{5}.agents = 9;
test{5}.IK.dist = [6.08; 7.38; 15.75; 14.87];
test{5}.IK.RMS = [54.23; 59.86; 50.99; 58.33];
test{5}.no_IK.dist = [11.17; 19.73; 13.17; 7.60];
test{5}.no_IK.RMS = [57.25; 60.14; 42.21; 52.78];

test{6}.agents = 11;
test{6}.IK.dist = [3.96; 8.57; 1.12];
test{6}.IK.RMS = [66.72; 52.76; 52.00];
test{6}.no_IK.dist = [22.31; 9.81; 3.99];
test{6}.no_IK.RMS = [68.88; 63.17; 57.84];

test{7}.agents = 13;
test{7}.IK.dist = [3.47; 5.55; 17.18];
test{7}.IK.RMS = [53.67; 66.60; 68.66];
test{7}.no_IK.dist = [12.94; 11.32; 18.34];
test{7}.no_IK.RMS = [57.32; 61.48; 68.58];

for i=1:7
  calc.IK.dist(i) = mean(test{i}.IK.dist);
  calc.IK.RMS(i) = mean(test{i}.IK.RMS);
  calc.no_IK.dist(i) = mean(test{i}.no_IK.dist);
  calc.no_IK.RMS(i) = mean(test{i}.no_IK.RMS);
end

% write the results of calc in a file as latex table in which each row is a test while the columns are the number of agents, the dist for IK and no_IK, the RMS for IK and no_IK
fid = fopen('results.tex','w');
% fprintf(fid,'\\begin{tabular}{cccccc}\n');
% fprintf(fid,'\\hline\n');
% fprintf(fid,'\\textbf{Test} & \\textbf{Agents} & \\textbf{IK dist} & \\textbf{no IK dist} & \\textbf{IK RMS} & \\textbf{no IK RMS} \\\\ \\hline\n');
for i=1:7
  fprintf(fid,'%d & %d & %.2f & %.2f & %.2f & %.2f \\\\\n',i,test{i}.agents,calc.IK.dist(i),calc.no_IK.dist(i),calc.IK.RMS(i),calc.no_IK.RMS(i));
end
fprintf(fid, '\\hline\n')
% fprintf(fid,'\\end{tabular}\n');
% fclose(fid);
