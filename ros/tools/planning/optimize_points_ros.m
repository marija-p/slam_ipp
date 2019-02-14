function obj = optimize_points_ros(waypoints, starting_point, yaw_init, Rob_P_init, ...
    field_map, occupancy_map, training_data, testing_data, ...
    map_params, planning_params, gp_params, transforms)
% Fitness function for optimizing all points on a horizon
% for an informative objective

waypoints = reshape(waypoints, 2, [])';
waypoints = [starting_point; waypoints];

obj = compute_objective_ros(waypoints, yaw_init, Rob_P_init, field_map, occupancy_map, ...
    training_data, testing_data, map_params, planning_params, gp_params, transforms);

end