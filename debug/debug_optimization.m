debug_file = 'debug1.mat';

load(debug_file)
compute_objective_debug(path_points, field_map, ...
    Rob, Sen, SimLmk, Lmk, Obs, Trj, Frm, Fac, factorRob, Opt, ...
    num_control_frames, currentFrame, training_data, testing_data, ...
    map_params, planning_params, gp_params)

load(debug_file)
compute_objective_debug(path_optimized, field_map, ...
    Rob, Sen, SimLmk, Lmk, Obs, Trj, Frm, Fac, factorRob, Opt, ...
    num_control_frames, currentFrame, training_data, testing_data, ...
    map_params, planning_params, gp_params)