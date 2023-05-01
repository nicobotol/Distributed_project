function [p_circle] = circle_sector(x_center, y_center, A, B)
% This function reports the point of a circle

step = 1e-2;
alpha_A = atan2(A(2) - y_center, A(1) - x_center);
alpha_B = atan2(B(2) - y_center, B(1) - x_center);

R = norm([x_center y_center] - A); % radius of the circle

th = alpha_A:step:alpha_B;
p_circle(:, 1) = x_center + R*cos(th);
p_circle(:, 2) = y_center + R*sin(th);
end