% Circle - radius, normal, and centre.
radius = 2.5; normal = [1,0,1.5]; center = [0,0,3];

% UAV workspace dimensions [m].
dim_x_env = 5;
dim_y_env = 5;
dim_z_env = 4;

% Load parameters.
[map_params, planning_params, opt_params, gp_params, ...
    training_data, gt_data, testing_data] = ...
    load_params_and_data(dim_x_env, dim_y_env, dim_z_env);
planning_params.control_freq = 2;
planning_params.meas_freq = 0.5;
planning_params.control_noise_coeffs = [0.02, 0.02, 0.02]*10^1;

% Circle trajectory
theta = pi/4:0.1:2*pi+pi/4;
v = null(normal);
traj_points = ...
    repmat(center',1,size(theta,2))+radius*(v(:,1)*cos(theta)+v(:,2)*sin(theta));
% scatter3(points(1,:),points(2,:),points(3,:));
% view(3)
% grid minor
% xlabel('x')
% ylabel('y')
% zlabel('z')
% axis([-2.5 2.5 -2.5 2.5 0 5])

rng(15, 'twister');
gp_params.use_modified_kernel = 0;
gp_params.N_gauss = 5;
[metrics] = slam_gp_circle(traj_points, map_params, planning_params, ...
    opt_params, gp_params, training_data, gt_data, testing_data);