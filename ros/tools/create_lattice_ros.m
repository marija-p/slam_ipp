function [lattice_env] = create_lattice_ros(map_params, planning_params)
% Create 2D lattice for TurtleBot3.

plot_lattice = 0;

dim_x_env = map_params.dim_x*map_params.res_x;
dim_y_env = map_params.dim_y*map_params.res_y;

[X,Y] = ...
    meshgrid(linspace(map_params.pos_x+map_params.res_x/2,map_params.pos_x+dim_x_env-map_params.res_x/2,planning_params.lattice_points_x), ...
    linspace(map_params.pos_y+map_params.res_y/2,map_params.pos_y+dim_y_env-map_params.res_y/2,planning_params.lattice_points_y));

lattice_env = [X(:), Y(:)];

if (plot_lattice)
    plot(lattice_env(:,1), lattice_env(:,2), '.k', 'MarkerSize', 20);
    axis([-2, 2, -2, 2])
end