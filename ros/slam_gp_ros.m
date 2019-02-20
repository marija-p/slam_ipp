function [metrics] = ...
    slam_gp_ros(map_params, planning_params, opt_params, gp_params, ...
    training_data, testing_data, transforms)
% TurtleBot3 temperature mapping experiments for RAL-19.
% ---
% M Popovic 2019
%

%clear

close all;

load map_lab_trim.mat
occupancy_map = map;
assignin('base', 'occupancy_map', occupancy_map);

% ROS communications
goal_pub = rospublisher('/move_base/goal');
goal_msg = rosmessage(goal_pub);
goal_msg.Goal.TargetPose.Header.FrameId = 'map';

scan_sub = rossubscriber('/scan');
temp_sub = rossubscriber('/temperature');
amcl_pose_sub = rossubscriber('/amcl_pose');

% Mapping parameters
dim_x = map_params.dim_x;
dim_y = map_params.dim_y;

% Lattice for 3D grid search
lattice = create_lattice_ros(map_params, planning_params);
% GP field map
field_map = [];

% Initialise data logging.
metrics = initialize_metrics(map_params, planning_params, opt_params, gp_params);

% Set initial measurement point.
points_meas = planning_params.meas_pose_init(1:2);
path_optimized = points_meas;

% Start timer.
time_elapsed = 0;
tic;

field_fig = create_field_fig(points_meas, map_params);

%% V. Main loop
while (true)
    
    % Check if planning budget exceeded.
    if (time_elapsed > planning_params.time_budget)
        disp('Time budget exceeded!')
        return;
    end
    
    %% Sensing.
    % Take measurements along path, updating the GP field map.
    for i = 1:size(points_meas)
        
        % Send the command.
        amcl_pose_msg = receive(amcl_pose_sub);
        point_current = [amcl_pose_msg.Pose.Pose.Position.X, ...
            amcl_pose_msg.Pose.Pose.Position.Y];
        goal_msg = create_goal_msg(point_current, points_meas(i,:), goal_msg);
        send(goal_pub, goal_msg);
        
        disp('Heading to: ')
        disp(points_meas(i,:))
        
        % Wait to reach target measurement point.
        reached_point = false;
        while (~reached_point)
            amcl_pose_msg = receive(amcl_pose_sub);
            point_current = [amcl_pose_msg.Pose.Pose.Position.X, ...
                amcl_pose_msg.Pose.Pose.Position.Y];
            if (pdist2(points_meas(i,:), point_current) < planning_params.achievement_dist)
                reached_point = true;
            end
        end
        
        % Get the robot position covariance.
        Rob_P = [amcl_pose_msg.Pose.Covariance(1), amcl_pose_msg.Pose.Covariance(2), amcl_pose_msg.Pose.Covariance(6); ...
            amcl_pose_msg.Pose.Covariance(7), amcl_pose_msg.Pose.Covariance(8), amcl_pose_msg.Pose.Covariance(12); ...
            amcl_pose_msg.Pose.Covariance(31), amcl_pose_msg.Pose.Covariance(32), amcl_pose_msg.Pose.Covariance(36)];
        % Ignore orientation for now. xD
        T_MAP_LINK = trvec2tform([point_current, 0]);
        T_MAP_TEMP = T_MAP_LINK * transforms.T_LINK_TEMP;
        x_MAP_TEMP = tform2trvec(T_MAP_TEMP);
        % Update the GP.
        pause(3);
        temp_msg = receive(temp_sub);
        temp = temp_msg.Data;
        
        [field_map, training_data] = ...
            take_measurement_at_point_ros(x_MAP_TEMP(1:2), Rob_P(1:2,1:2), temp, field_map, ...
            training_data, testing_data, gp_params);
        
        current_time = toc;
        metrics.times = [metrics.times; current_time];
        metrics.points_meas = [metrics.points_meas; point_current];
        metrics.measurements = [metrics.measurements; training_data.Y_train(end)];
        metrics.P_traces = [metrics.P_traces; sum(field_map.cov)];
        metrics.Rob_Ps(:,:,size(metrics.times,1)) = Rob_P;
        
        % Update graphics
        field_fig = ...
            update_field_fig(field_fig, field_map, path_optimized, points_meas, map_params);
        
        %keyboard
        
    end
    
    %% Planning.
    % I. Grid search.
    q = [amcl_pose_msg.Pose.Pose.Orientation.W, amcl_pose_msg.Pose.Pose.Orientation.X, ...
        amcl_pose_msg.Pose.Pose.Orientation.Y, amcl_pose_msg.Pose.Pose.Orientation.Z];
    rot = quat2eul(q);
    pose_current = [point_current, rot(3)];
    path_points = search_lattice_ros(pose_current, Rob_P, lattice, ...
        field_map, occupancy_map, training_data, testing_data, ...
        map_params, gp_params, planning_params, transforms);
    
    disp('Lattice search result: ')
    disp(path_points)
    
    % II. Trajectory optimization.
    if (strcmp(opt_params.opt_method, 'cmaes'))
        path_optimized = optimize_with_cmaes_ros(path_points, rot(3), Rob_P, ...
            field_map, occupancy_map, training_data, testing_data, ...
            map_params, planning_params, opt_params, gp_params, transforms);
    else
        path_optimized = points_path;
    end
    
    disp('Optimization result: ')
    disp(path_optimized)
    %disp(['Time: ', num2str(Map.t)])
    keyboard
    
    % Interpolate measurements along the path.
    points_path_steps = [0; sqrt(sum(diff(path_optimized,[],1).^2,2))]';
    points_path_cumlen = cumsum(points_path_steps);
    tq = 0:planning_params.max_vel/planning_params.meas_freq:points_path_cumlen(end);
    points_meas = [];
    points_meas(:,1) = interp1(points_path_cumlen, path_optimized(:,1)', tq, 'linear');
    points_meas(:,2) = interp1(points_path_cumlen, path_optimized(:,2)', tq, 'linear');
 
    metrics.path_travelled = [metrics.path_travelled; path_optimized];
   
    save test_trial.mat                                                

    disp('Measurement points: ')
    disp(points_meas)
    
end
