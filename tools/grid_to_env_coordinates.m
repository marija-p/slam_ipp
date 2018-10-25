function [point_env] = grid_to_env_coordinates(point_grid, map_params)
% Convert coordinates from grid map to environment representation.

point_env = point_grid;
% 2D: Subtract half a cell size in the conversion
% to get to the center of the cell.
point_env(:,1) = point_env(:,1) * map_params.res_x + ...
    map_params.pos_x - map_params.res_x/2;
point_env(:,2) = point_env(:,2) * map_params.res_y + ...
    map_params.pos_y - map_params.res_y/2;

if isfield(map_params, 'res_z')
    % 3D: Treat as discrete altitude levels.
    point_env(:,3) = (point_env(:,3) - 1 + map_params.pos_z) * map_params.res_z;
end
end