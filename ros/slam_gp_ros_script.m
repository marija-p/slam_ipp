% Workspace dimensions [m]
dim_x_env = 2.8;
dim_y_env = 2.8;

% Static transforms.
transforms.T_LINK_TEMP = ...              % Robot body -> temperature sensor.
    [1, 0, 0, -0.182;
     0, 1, 0, 0.0630;
     0, 0, 1, 0.122;
     0, 0, 0, 1];
 
transforms.T_LINK_SCAN = ...              % Robot body -> laser scanner.
     [1, 0, 0, -0.064;
     0, 1, 0, 0.0;
     0, 0, 1, 0.122;
     0, 0, 0, 1];
 
 % Parameters
 [map_params, planning_params, opt_params, gp_params, training_data, testing_data] = ...
     load_params_and_data_ros(dim_x_env, dim_y_env);

% Let'ross go!
[metrics] = slam_gp_ros(map_params, planning_params, opt_params, gp_params, ...
    training_data, testing_data, transforms);