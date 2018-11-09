function [map_params, planning_params, opt_params, gp_params, ...
    training_data, gt_data, testing_data] = ...
    load_params_and_data(dim_x_env, dim_y_env, dim_z_env)
% Loads default parameters + training data for RA-L19 IPP algorithms.

load training_data_3d.mat

% Map resolution [m/cell]
map_params.res_x = 0.5;
map_params.res_y = 0.5;
map_params.res_z = 1;
% Map dimensions [cells]
map_params.dim_x = dim_x_env/map_params.res_x;
map_params.dim_y = dim_y_env/map_params.res_y;
map_params.dim_z = dim_z_env/map_params.res_z;
% Position of map in the environment [m]
map_params.pos_x = -dim_x_env / 2;
map_params.pos_y = -dim_y_env / 2;
map_params.pos_z = 1;

% Number of points in the 3D lattice
planning_params.lattice_points_x = 3;
planning_params.lattice_points_y = 3;
planning_params.lattice_points_z = 3;

% Trajectory optimization references
planning_params.max_vel = 3;        % [m/s]
planning_params.max_acc = 3;        % [m/s^2]

% Frequency at which to send commands - simulate motion/covariance update
planning_params.control_freq = 3;   % [Hz]
% Frequency at which to take measurements
planning_params.meas_freq = 0.2;    % [Hz]
% Factor by which to divide keyframe/graph optimisation frequency
% (Opt.map.kfrmPeriod).
planning_params.keyframe_predict_factor = 1.5;

% Number of control points for a polynomial (start point fixed).
planning_params.control_points = 3;

% Optimization/CMA-ES related parameters
opt_params.max_iters = 15;
opt_params.opt_method = 'cmaes'; % 'fmc'/cmaes'/'none'/'bo'
% Covariances in each search dimension
opt_params.cov_x = 4;
opt_params.cov_y = 4;
opt_params.cov_z = 1.5;

% GP-related parameters
gp_params.hyp_trained = hyp_trained;
gp_params.N_gauss = 4; %N_gauss;
gp_params.inf_func = inf_func;
gp_params.mean_func = mean_func;
gp_params.inf_func = inf_func;
gp_params.cov_func = cov_func;
gp_params.lik_func = lik_func;

% Ground truth data - scale for this environment.
gt_data.X_gt(:,1) = X_gt(:,1) + map_params.pos_x;
gt_data.X_gt(:,2) = X_gt(:,2) + map_params.pos_y;
gt_data.X_gt(:,3) = X_gt(:,3) + map_params.pos_z;
gt_data.Y_gt = Y_gt;
% Testing data - same resolution as ground truth
testing_data.X_test = gt_data.X_gt;
% Training data - estimated and real (ground truth)
training_data.X_train = [];
training_data.P_train = zeros(3,3,200);
training_data.X_train_gt = [];
training_data.Y_train = [];

end