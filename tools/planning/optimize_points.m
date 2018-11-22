function obj = optimize_points(waypoints, starting_point, field_map, ...
    Rob, Sen, SimLmk, Lmk, Obs, Trj, Frm, Fac, factorRob, Opt, ...
    num_control_frames, training_data, testing_data, ...
    map_params, planning_params, gp_params)
% Fitness function for optimizing all points on a horizon
% for an informative objective

% Important: remember global variable to restore later.
global Map
Map_init = Map;

waypoints = reshape(waypoints, 3, [])';
waypoints = [starting_point; waypoints];

obj = compute_objective(waypoints, field_map, ...
    Rob, Sen, SimLmk, Lmk, Obs, Trj, Frm, Fac, factorRob, Opt, ...
    num_control_frames, training_data, testing_data, ...
    map_params, planning_params, gp_params);

Map = Map_init;

end