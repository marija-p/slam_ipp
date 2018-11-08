function path_optimized = optimize_with_cmaes(path, field_map, ...
    map_params, planning_params, opt_params)
% Optimizes a polynomial path (defined by control points) using
% the Covariance Matrix Adaptation Evolutionary Strategy (CMA-ES).
% ---
% M Popovic 2018
%

dim_x_env = map_params.dim_x*map_params.res_x;
dim_y_env = map_params.dim_y*map_params.res_y;

% Set optimization parameters.
opt = cmaes;
opt.DispFinal = 'off';
opt.LogModulo = 0;
opt.TolFun = 1e-9;
opt.IncPopSize = 1; %% Check this
opt.PopSize = 25;
opt.SaveVariables = 'off';
opt.MaxIter = opt_params.max_iters;
opt.Seed = randi(2^10);

% Set bounds and covariances.
LBounds = [-dim_x_env/2;-dim_y_env/2;planning_params.min_height];
UBounds = [dim_x_env/2;dim_y_env/2;planning_params.max_height];
opt.LBounds = repmat(LBounds, size(path,1)-1, 1);
opt.UBounds = repmat(UBounds, size(path,1)-1, 1); 
cov = [opt_params.cov_x; opt_params.cov_y; opt_params.cov_z];
cov = repmat(cov, size(path,1)-1, 1);

% Remove starting point (as this is fixed).
path_initial = reshape(path(2:end,:)', [], 1);
path_optimized = cmaes('optimize_points', path_initial, cov, opt, path(1,:), ...
    field_map, map_params, planning_params);
path_optimized = reshape(path_optimized, 3, [])';
path_optimized = [path(1,:); path_optimized];

end