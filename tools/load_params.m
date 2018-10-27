function [planning_params, map_params] = ...
    load_params(dim_x_env, dim_y_env, dim_z_env)
% Loads default parameters for RA-L19 IPP algorithms.

% Map resolution [m/cell]
map_params.res_x = 0.5;
map_params.res_y = 0.5;
map_params.res_z = 1;
% Map dimensions [cells]
map_params.dim_x = dim_x_env/map_params.res_x;
map_params.dim_y = dim_y_env/map_params.res_y;
map_params.dim_z = dim_z_env/map_params.res_z;
% Position of map in the environment [m]
map_params.pos_x = -dim_x_env / 2;
map_params.pos_y = -dim_y_env / 2;
map_params.pos_z = 1;

planning_params.lattice_points_x = 3;
planning_params.lattice_points_y = 3;
planning_params.lattice_points_z = 3;

end

