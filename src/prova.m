clear 
clc
ag{1}.x = [1 2 3 4 5;1 2 3 4 5;1 2 3 4 5];
ag{1}.P_est = cell(5,1);
ag{1}.P_est{1} = 10*eye(3);
ag{1}.P_est{2} = 100*eye(3);
ag{1}.P_est{3} = 100*eye(3);
ag{1}.P_est{4} = 100*eye(3);
ag{1}.P_est{5} = 100*eye(3);
for i=2:5
ag{i}.x=100*ones(3,5);
ag{i}.P_est = cell(5,1);
for j=1:5
  ag{i}.P_est{j} = 100*eye(3);
end

end

ag = wls(ag);
ag{1}.global_centroid