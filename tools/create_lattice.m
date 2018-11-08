function [lattice_env] = create_lattice(map_params, planning_params)
% Create multi-dimensional lattice in the UAV workspace.

plot_lattice = 0;

dim_x_env = map_params.dim_x*map_params.res_x;
dim_y_env = map_params.dim_y*map_params.res_y;
dim_z_env = map_params.dim_z*map_params.res_z;

[X,Y,Z] = ...
    meshgrid(linspace(map_params.pos_x+map_params.res_x/2,map_params.pos_x+dim_x_env-map_params.res_x/2,planning_params.lattice_points_x), ...
    linspace(map_params.pos_y+map_params.res_y/2,map_params.pos_y+dim_y_env-map_params.res_y/2,planning_params.lattice_points_y), ...
    linspace(map_params.pos_z+map_params.res_z/2,map_params.pos_z+dim_z_env-map_params.res_z/2,planning_params.lattice_points_z));

lattice_env = [X(:), Y(:), Z(:)];

if (plot_lattice)
    plot3(lattice_env(:,1), lattice_env(:,2), lattice_env(:,3), '.k', 'MarkerSize', 20);
    axis([-10, 10, -10, 10, 0, 10])
end