function [p_circle] = circle_sector(x_center, y_center, A, B)
% This function reports the point of a circle

step = 1e-2;
% wrapTo2Pi is used to have the angle between 0 and 2*pi
alpha_A = wrapTo2Pi(atan2(A(2) - y_center, A(1) - x_center)); 
alpha_B = wrapTo2Pi(atan2(B(2) - y_center, B(1) - x_center));

% A is the point at the left of the agent, and B at the right
% take into account if the the alpha_B is smaller than alpha_A -> add 2*pi in order to have it bigger
% NOTE: we want to go always from A to B in order to have the circular sector that include the agent
if alpha_B < alpha_A 
  alpha_B = alpha_B + 2*pi;
end

% compute the radius of the sensing range inside the function in order to avoid numerical issues
R = norm([x_center y_center]' - A); % radius of the circle

% build the arc
% Attention in adding always the final point and consider it only once
th = [alpha_A:step:alpha_B, alpha_B];
th = unique(th);
p_circle(:, 1) = x_center + R*cos(th);
p_circle(:, 2) = y_center + R*sin(th);
end