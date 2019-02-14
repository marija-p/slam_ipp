%close all; clear all;

load map.mat

planning_params.control_freq = 15;
planning_params.max_vel = 0.25;

% Interpolate measurement points along the path.
points_path = [0, 0; 0, 2.5];
points_path_steps = [0; sqrt(sum(diff(points_path,[],1).^2,2))]';
points_path_cumlen = cumsum(points_path_steps);
tq = 0:planning_params.max_vel/planning_params.control_freq:points_path_cumlen(end);
points_meas = [];
points_meas(:,1) = interp1(points_path_cumlen, points_path(:,1)', tq, 'spline');
points_meas(:,2) = interp1(points_path_cumlen, points_path(:,2)', tq, 'spline');

%% Lidar scanner.
lidar = LidarSensor;
lidar.mapName = 'map';
lidar.sensorOffset = [0,0];
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
rangeFinderModel.Map = map;

% Transform: /base_link -> /camera_depth_frame
rangeFinderModel.SensorPose = [0, 0, 0];

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
amcl.InitialPose = start_pose;
amcl.InitialCovariance = eye(3)*0.5;

%% Control loop.
for i = 2:size(points_meas,1)
 
    % MOTION: differential drive.
    current_pose = [points_meas(i,1:2), ...
        get_yaw(points_meas(i-1,1:2), points_meas(i,1:2))];

    % MEASUREMENT: lidar scan.
    % Simulate scan.
    ranges = lidar(current_pose');
    %plot(lidar.scanAngles,ranges,'o-');
    scan = lidarScan(ranges, lidar.scanAngles);
    
    % Update estimated robot's pose and covariance using new odometry and
    % sensor readings.
    [isUpdated, estimatedPose, estimatedCovariance] = amcl(current_pose, scan);

    disp(current_pose)
    
end

disp(estimatedPose)
disp(estimatedCovariance)