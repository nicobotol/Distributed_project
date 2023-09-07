function [cone, len_cone, dy] = feedback_motion_prediction_chute(theta, x, y)
% This function computes the area where the unicycle can move
% theta -> angle of the parachute wrt the world frame
% x -> 2D position of the chute
% y -> 2D target point
% cone -> polyshape in case the starting and ending points are aligned
% len_cone -> 1 if cone is an area, 0 if it is a segment

% Problem when collapsing the circle to a point (the cone becomes a line)
dy = norm([-sin(theta) cos(theta)]*(y - x)');

if dy > 1e-8
  % Circle centerd in y with radius equal to the goal alignment distance
  B = circle(y(1), y(2), dy);
  B_shape = polyshape(B(:,1), B(:,2));

  w = x + ([cos(theta) sin(theta)]'*[cos(theta) sin(theta)]*(y - x)')';

  % Points for the convex hull operator
  z = [x; y; w];

  hull = convhull(z(:,1), z(:,2));

  % Truncated cone
  shape = polyshape(z(hull,1), z(hull,2));
  cone = union(shape, B_shape);
  len_cone = size(cone.Vertices, 1);
else 
  cone = [x; y]; % segment starting in the actual state and ending in the target
  len_cone = -1; % write -1 in order to know that the cone is a segment
end

end
