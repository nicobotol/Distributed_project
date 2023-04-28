function [p_circle] = circle_sector(x_center, y_center, A, B)
% This function reports the point of a circle

step = 1e-2;
alpha_A = wrapTo2Pi(atan2(A(2) - y_center, A(1) - x_center));
alpha_B = wrapTo2Pi(atan2(B(2) - y_center, B(1) - x_center));

R = norm([x_center y_center] - A); % radius of the circle
if alpha_B - alpha_A > pi
  th = alpha_A:step:alpha_B;
else 
  th = alpha_B:step:alpha_A;
end

p_circle(:, 1) = x_center + R*cos(th);
p_circle(:, 2) = y_center + R*sin(th);
end