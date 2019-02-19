plot_trajectory = 0;

% UAV workspace dimensions [m].
dim_x_env = 5;
dim_y_env = 5;
dim_z_env = 4;

% Load parameters.
[map_params, planning_params, opt_params, gp_params, ...
    training_data, gt_data, testing_data] = ...
    load_params_and_data(dim_x_env, dim_y_env, dim_z_env);
planning_params.max_vel = 5;
planning_params.max_acc = 5;
planning_params.control_freq = 2;
planning_params.meas_freq = 0.5;
planning_params.control_noise_coeffs = [0.01, 0.01, 0.01]*0.6;

% Inclined circle trajectory
%radius = 2.5; normal = [1,0,1.5]; center = [0,0,3];
%theta = pi/4:0.1:2*pi+pi/4;
%v = null(normal);
%traj_points = ...
%    repmat(center',1,size(theta,2))+radius*(v(:,1)*cos(theta)+v(:,2)*sin(theta));

% Spiral trajectory
radius = 2.3; height = 2;
t = -pi/2:0.1:3*pi + pi/2;
x = radius * sin(t);
y = radius * cos(t);
z = 1.3 + height/(2*pi) * t;
traj_points = [x;y;z];

if (plot_trajectory)
    scatter3(traj_points(1,:),traj_points(2,:),traj_points(3,:));
    view(3)
    grid minor
    xlabel('x')
    ylabel('y')
    zlabel('z')
    axis([-2.5 2.5 -2.5 2.5 0 5])
    keyboard
end

rng(6, 'twister'); % 1 - without UI, 6 - with UI
gp_params.use_modified_kernel = 1;
gp_params.N_gauss = 5;
[metrics] = slam_gp_circle(traj_points, map_params, planning_params, ...
    opt_params, gp_params, training_data, gt_data, testing_data);