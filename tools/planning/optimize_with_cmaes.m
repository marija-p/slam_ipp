function path_optimized = optimize_with_cmaes(path_points, field_map, ...
    Rob, Sen, SimLmk, Lmk, Obs, Trj, Frm, Fac, factorRob, Opt, num_control_frames, current_frame, ...
    training_data, testing_data, map_params, planning_params, opt_params, gp_params)
% Optimizes a polynomial path (defined by control points) using
% the Covariance Matrix Adaptation Evolutionary Strategy (CMA-ES).
% ---
% M Popovic 2018
%

dim_x_env = map_params.dim_x*map_params.res_x;
dim_y_env = map_params.dim_y*map_params.res_y;
dim_z_env = map_params.dim_z*map_params.res_z;

% Set optimization parameters.
opt = cmaes;
opt.DispFinal = 'off';
opt.LogModulo = 0;
opt.TolFun = 1e-9;
opt.IncPopSize = 1; %% Check this
opt.PopSize = 25;
opt.SaveVariables = 'off';
opt.MaxIter = opt_params.max_iters;
opt.Seed = randi(2^5);

% Set bounds and covariances.
LBounds = [map_params.pos_x;map_params.pos_y;map_params.pos_z];
UBounds = [map_params.pos_x+dim_x_env;map_params.pos_y+dim_y_env;map_params.pos_z+dim_z_env];
opt.LBounds = repmat(LBounds, size(path_points,1)-1, 1);
opt.UBounds = repmat(UBounds, size(path_points,1)-1, 1);
cov = [opt_params.cov_x; opt_params.cov_y; opt_params.cov_z];
cov = repmat(cov, size(path_points,1)-1, 1);

% Remove starting point (as this is fixed).
path_initial = reshape(path_points(2:end,:)', [], 1);
path_optimized = cmaes('optimize_points', ...
    path_initial, cov, opt, path_points(1,:), field_map, ...
    Rob, Sen, SimLmk, Lmk, Obs, Trj, Frm, Fac, factorRob, Opt, ...
    num_control_frames, current_frame, ...
    training_data, testing_data, ...
    map_params, planning_params, gp_params);
path_optimized = reshape(path_optimized, 3, [])';
path_optimized = [path_points(1,:); path_optimized];

end