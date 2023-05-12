function [p_circle] = circle(x_center, y_center, R)
% This function reports the point of a circle

step = 1e-2;
th = step:step:2*pi; % angle 
p_circle(:, 1) = x_center + R*cos(th);
p_circle(:, 2) = y_center + R*sin(th);
end