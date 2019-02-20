function [map_params, planning_params, opt_params, gp_params, ...
    training_data, testing_data] = ...
    load_params_and_data_ros(dim_x_env, dim_y_env)
% Loads default parameters + training data for RA-L19 IPP algorithms.

load hyp_trained.mat

% Map resolution [m/cell]
map_params.res_x = 0.40;
map_params.res_y = 0.40;
% Map dimensions [cells]
map_params.dim_x = round(dim_x_env/map_params.res_x);
map_params.dim_y = round(dim_y_env/map_params.res_y);
% Position of map in the environment [m]
map_params.pos_x = -dim_x_env / 2;
map_params.pos_y = -dim_y_env / 2;

% Number of points in the 3D lattice
planning_params.lattice_points_x = 3;
planning_params.lattice_points_y = 3;

% Trajectory optimization references
planning_params.max_vel = 0.26;        % [m/s]

% Robot initial measurement pose [x,y,yaw] [m,m,rad]
planning_params.meas_pose_init = [0.2, 0, 0];

% Achievement distance before a point is considered reached [m].
planning_params.achievement_dist = 0.05;

% Frequency at which to take measurements (real)
planning_params.meas_freq = 0.25;    % [Hz]
% Frequency at which to simulate control actions (uncertainty prediction)
planning_params.control_freq = 2;   % [Hz]

% Number of control points for a polynomial (start point fixed).
planning_params.control_points = 3;

% Total time budget for informative planning [s].
planning_params.time_budget = 180;

% Objective function for informative planning.
% 'uncertainty_adaptive'/'uncertainty_rate_adaptive'/'renyi_adaptive'
% 'uncertainty'/'uncertainty_rate'/'renyi'
planning_params.obj_func = 'renyi_adaptive';
% Renyi objective function only: uncertainty measure for alpha parameter.
planning_params.renyi_uncertainty = 'Aopt';

% Threshold for adaptive planning - only regions above this value are
% considered "interesting" and used when computing information gain.
planning_params.lower_thres = 27;
% Design parameter for uncertainty-aware adaptive planning
planning_params.beta = 1;

% Optimization/CMA-ES related parameters
opt_params.max_iters = 3;
opt_params.opt_method = 'cmaes'; % 'fmc'/cmaes'/'none'/'bo'
% Covariances in each search dimension
opt_params.cov_x = 2;
opt_params.cov_y = 2;

% GP-related parameters
hyp_trained.mean = 28;
gp_params.hyp_trained = hyp_trained;
gp_params.N_gauss = 9; %N_gauss;
gp_params.inf_func = inf_func;
gp_params.mean_func = mean_func;
gp_params.inf_func = inf_func;
gp_params.cov_func = cov_func;
gp_params.lik_func = lik_func;
% Whether to account for robot's pose uncertainty in GP field mapping.
gp_params.use_modified_kernel = 1;
% Whether to account for robot's pose uncertainty in prediction - objective function.
gp_params.use_modified_kernel_prediction = 0;

% Create the inference grid.
x = 0:map_params.res_x:dim_x_env-map_params.res_x;
y = 0:map_params.res_y:dim_y_env-map_params.res_y;
[X,Y] = meshgrid(x,y); mesh = [X(:) Y(:)];
testing_data.X_test = mesh;
testing_data.X_test(:,1) = testing_data.X_test(:,1) + map_params.pos_x;
testing_data.X_test(:,2) = testing_data.X_test(:,2) + map_params.pos_y;
% Training data - estimated and real (ground truth)
training_data.X_train = [];
training_data.P_train = zeros(2,2,200);
training_data.Y_train = [];

end