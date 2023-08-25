x = [0 1 3];
y = [0 0 1];

bs = [-20 -20; 20 -20; 20 20; -20 20];

[V,C,XY] = VoronoiLimit(x, y, 'bs_ext', bs, 'figure', 'off');

figure(); hold on;
for i = 1:3
plot(polyshape(V(C{i}, 1), V(C{i}, 2)));
plot(x(i), y(i), 'rx', 'MarkerSize', 20);
end