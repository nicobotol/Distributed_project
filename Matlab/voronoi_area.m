function lim_voronoi = voronoi_area(p, v_voronoi, Rs, fake_inf)
% This function returns the voronoi area in different cases
% p -> positon of the cell center
% v_voronoi -> vertices of the voronoi tessellation for the agent
% Rs -> sensing range
% fake_inf -> fake infinite
[row, ~] = size(v_voronoi);
switch row
  case 1
    circ = circle(p(1), p(2), Rs);
    lim_voronoi = polyshape(circ);
  case 2
    % Intersection between a circle with radius rs and a rectangle with
    % width rs+l/2 and height 2rs, with l equal to the differnce between
    % the agents
    [~, pos] = min(v_voronoi);         % voronoi point
    p_v = v_voronoi(pos(1), :);
    r = norm(p - p_v);            % radius of the circle 
    circ = circle(p(1), p(2), r);
    lim_voronoi = polyshape(circ);
  case 3
    if ~isinf(v_voronoi(:, 1))
      lim_voronoi = polyshape(v_voronoi(:, 1), v_voronoi(:, 2));
    else
      [~, inf_pos] = max(v_voronoi); % position of the infinite
      v_tmp = zeros(size(v_voronoi));
      v_tmp(:, :) = v_voronoi;
      v_tmp(inf_pos(1), :) = [];
      p_mean = mean(v_tmp);             % middle point
      dir = p_mean - p;   % direction
      dir = -dir/norm(dir);
      v_voronoi(inf_pos(1), :) = fake_inf*dir + p;
      lim_voronoi = polyshape(v_voronoi(:, 1), v_voronoi(:, 2));
    end 
  case 4
    if ~isinf(v_voronoi(:, 1))
      lim_voronoi = polyshape(v_voronoi(:, 1), v_voronoi(:, 2));
    else
      [~, inf_pos] = max(v_voronoi); % position of the infinite
      v_tmp = zeros(2);
      if inf_pos == 1
        v_tmp(1, :) = v_voronoi(2, :);
        v_tmp(2, :) = v_voronoi(end, :);
      elseif inf_pos == length(v_voronoi)
        v_tmp(1, :) = v_voronoi(1, :);
        v_tmp(2, :) = v_voronoi(end-1, :);
      else
        v_tmp(1, :) = v_voronoi(inf_pos(1) - 1, :);
        v_tmp(2, :) = v_voronoi(inf_pos(1) + 1, :);
      end
      p_mean = mean(v_tmp);             % middle point
      dir = p_mean - p;   % direction
      dir = -dir/norm(dir);
      v_voronoi(inf_pos(1), :) = fake_inf*dir + p;
      lim_voronoi = polyshape(v_voronoi(:, 1), v_voronoi(:, 2));
    end

  otherwise
    lim_voronoi = polyshape(v_voronoi(:, 1), v_voronoi(:, 2));
end
end