function [field_map, Rob_P] = ...
    predict_path_ros(path_points, yaw_init, Rob_P_init, ...
    field_map, occupancy_map, training_data, testing_data, ...
    map_params, gp_params, planning_params, transforms)
% Predicts robot uncertainty (using AMCL) + field map for a candidate path.
% NB: - Stuff is hard-coded for TurtleBot3. No time to make parameters. xD
% ---
% Inputs:
% path_point: candidate path
% yaw_init: initial robot yaw [rad]
% Rob_P_init: initial robot pose covariance (3x3)
% field_map: current GP map (mean + covariance)
% occupancy_map: gmapping occupancy grid output
% ---
% Output:
% field_map: final GP map (mean + covariance)
% Rob_P: final robot pose covariance (3x3)
% ---
% M Popovic 2019
%

meas_frame_interval = planning_params.control_freq/planning_params.meas_freq;

% Interpolate the path.
points_path_steps = [0; sqrt(sum(diff(path_points,[],1).^2,2))]';
points_path_cumlen = cumsum(points_path_steps);
tq = 0:planning_params.max_vel/planning_params.control_freq:points_path_cumlen(end);
points_control = [];
points_control(:,1) = interp1(points_path_cumlen, path_points(:,1)', tq, 'spline');
points_control(:,2) = interp1(points_path_cumlen, path_points(:,2)', tq, 'spline');

%% Lidar scanner.
lidar = LidarSensor;
lidar.mapName = 'occupancy_map';
lidar.sensorOffset = [transforms.T_LINK_SCAN(1,4), transforms.T_LINK_SCAN(2,4)];
lidar.scanAngles = linspace(-pi,pi,360);
lidar.maxRange = 3.5;

%% Localization system.
% Motion model.
odometryModel = robotics.OdometryMotionModel;
% Rotational + translational errors.
odometryModel.Noise = [0.2 0.2 0.2 0.2];

% Measurement model.
rangeFinderModel = robotics.LikelihoodFieldSensorModel;
% Maximum and minimum range of sensor.
rangeFinderModel.SensorLimits = [0.12 lidar.maxRange];
rangeFinderModel.Map = occupancy_map;

% Transform: /base_link -> /camera_depth_frame
rangeFinderModel.SensorPose = ...
    [transforms.T_LINK_SCAN(1,4), transforms.T_LINK_SCAN(2,4), 0];

% Set up Monte Carlo Localization.
amcl = robotics.MonteCarloLocalization;
amcl.UseLidarScan = true;
amcl.MotionModel = odometryModel;
amcl.SensorModel = rangeFinderModel;

% Set parameters.
amcl.UpdateThresholds = [0.2, 0.2, 0.2];
amcl.ResamplingInterval = 1;
% Lower and upper bound on the number of particles generated during
% resampling.
amcl.ParticleLimits = [500 5000];
amcl.GlobalLocalization = false;
% Gaussian distribution for initial pose.
amcl.InitialPose = [path_points(1,:), yaw_init];
amcl.InitialCovariance = Rob_P_init;

%% Control loop.
for i = 2:size(points_control,1)
 
    % MOTION: differential drive.
    current_pose = [points_control(i,1:2), ...
        get_yaw(points_control(i-1,1:2), points_control(i,1:2))];

    % MEASUREMENT: lidar scan.
    % Simulate scan.
    ranges = lidar(current_pose');
    %plot(lidar.scanAngles,ranges,'o-');
    scan = lidarScan(ranges, lidar.scanAngles);
    
    % Update estimated robot's pose and covariance using new odometry and
    % sensor readings.
    [~, pose_est, Rob_P] = amcl(current_pose, scan);
    
    if (mod(i-1,meas_frame_interval) == 0)
        %disp('Predicting measurement at: ')
        %disp(pose_est(1:2))
        field_map = predict_map_update_ros(pose_est(1:2), Rob_P(1:2, 1:2), ...
            field_map, training_data, testing_data, map_params, gp_params);
    end
    
end

%disp(Rob_P)

end

