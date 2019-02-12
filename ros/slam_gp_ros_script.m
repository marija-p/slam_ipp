% Workspace dimensions [m]
dim_x_env = 5;
dim_y_env = 5;

% Static transforms.
transforms.T_LINK_TEMP = ...              % Robot body -> temperature sensor.
    [1, 0, 0, -0.182;
     0, 1, 0, 0.0630;
     0, 0, 1, 0.122;
     0, 0, 0, 1];
 
[map_params, planning_params, opt_params, gp_params, training_data, testing_data] = ...
    load_params_and_data_ros(dim_x_env, dim_y_env);

[metrics] = slam_gp_ros(map_params, planning_params, opt_params, gp_params, ...
    training_data, testing_data, transforms);