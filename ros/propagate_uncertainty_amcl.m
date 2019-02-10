close all; clear all;

load map.mat

start_pose = [0; 0; pi/2];
goal_pose = [0; 2.5; pi/2];

%% Robot.
robot_radius = 0.4;
robot = ExampleHelperRobotSimulator('emptyMap',2);
robot.enableLaser(false);
robot.setRobotSize(robot_radius);
robot.showTrajectory(true);
robot.setRobotPose(start_pose);

%% Controller.
% Pure pursuit.
path = [start_pose'; goal_pose'];
controller = robotics.PurePursuit;
controller.Waypoints = path(:,1:2);
controller.DesiredLinearVelocity = 0.3;
controller.MaxAngularVelocity = 2;
controller.LookaheadDistance = 0.5;

goalRadius = 0.1;
distanceToGoal = norm(start_pose(1:2) - goal_pose(1:2));
controlRate = robotics.Rate(10);

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
while( distanceToGoal > goalRadius )
    
    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(robot.getRobotPose);
    
    % Simulate the robot using the controller outputs.
    drive(robot, v, omega);
 
    % MOTION: differential drive.
    current_pose = robot.getRobotPose;
    
    % MEASUREMENT: lidar scan.
    % Simulate scan.
    ranges = lidar(current_pose');
    %plot(lidar.scanAngles,ranges,'o-');
    scan = lidarScan(ranges, lidar.scanAngles);
    
    % Update estimated robot's pose and covariance using new odometry and
    % sensor readings.
    [isUpdated, estimatedPose, estimatedCovariance] = amcl(current_pose, scan);
    
    % Re-compute the distance to the goal.
    distanceToGoal= norm(current_pose(1:2)' - goal_pose(1:2));
    
    waitfor(controlRate);
    disp(current_pose)
    
end

disp(estimatedPose)
disp(estimatedCovariance)