function [A, B] = v_voronoi2pt(O1, O2, Rs)
% Find the points that delimits the voronoi semicircle incase of only 2
% agents in the sensing range
% O1 and O2 are the positions of the voronoi agents
% Rs is the sensing range radius

l = O2 - O1;
r = (l)/norm(l);
d = sqrt(Rs^2 - (norm(l)/2)^2);
M = mean([O1, O2], 2);
c = null(r.');
A = M + c*d;
B = M - c*d;

end