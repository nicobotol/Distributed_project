P = [0.5 0; 0 0.5; -0.5 -0.5; -0.2 -0.1; -0.1 0.1; 0.1 -0.1; 0.1 0.1];
[v, c] = voronoin(P);
figure()
plot(v(:,1), v(:,2), 'o', 'Color', 'r')
hold on
voronoi(P(:,1), P(:,2))