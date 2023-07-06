function [C] = convexhull(x, meshed_B)
  % This function reports the point of a circle

  % x, y: the coordinates of the points
  % ro: the radius of the circle

  C = [];
  
  for i=1:length(meshed_B.Nodes)
    for alpha=0:0.005:1 shape.Vertices = [shape.Vertices; [x(1) x(2)]];
      z = meshed_B.Nodes(:,i);
      C = [C, x + alpha*(z-x)];
    end
  end
  end