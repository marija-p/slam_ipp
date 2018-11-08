function obj = optimize_points(waypoints, starting_point, field_map, ...
     map_params, planning_params)
% Fitness function for optimizing all points on a horizon
% for an informative objective

waypoints = reshape(waypoints, 3, [])';
waypoints = [starting_point; waypoints];

obj = compute_objective(waypoints, field_map, map_params, planning_params);

end