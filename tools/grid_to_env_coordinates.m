function [point_env] = grid_to_env_coordinates(point_grid, map_params)
% Convert coordinates from grid map to environment representation.

point_env = point_grid;
% 2D: Subtract half a cell size in the conversion
% to get to the center of the cell.
point_env(:,1) = point_env(:,1) * map_params.res_x + ...
    map_params.position_x - map_params.res_x/2;
point_env(:,2) = point_env(:,2) * map_params.res_y + ...
    map_params.position_y - map_params.res_y/2;

end