num_trials = 1;

% UAV workspace dimensions [m]
dim_x_env = 12;
dim_y_env = 12;
dim_z_env = 5;

for trial = 1:num_trials
    
    [map_params, planning_params, opt_params, gp_params, ...
        training_data, gt_data, testing_data] = ...
        load_params_and_data(dim_x_env, dim_y_env, dim_z_env);
    
    [metrics] = slam_gp(map_params, planning_params, opt_params, gp_params, ...
        training_data, gt_data, testing_data);
    
    clear;
    
end