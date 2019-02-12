function [metrics] = ...
    slam_gp_ros(map_params, planning_params, opt_params, gp_params, ...
    training_data, testing_data, transforms)
% TurtleBot3 temperature mapping experiments for RAL-19.
% ---
% M Popovic 2019
%

%clear

load map.mat

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
%lattice = create_lattice(map_params, planning_params);
% GP field map
field_map = [];

% Initialise data logging.
metrics = initialize_metrics(map_params, planning_params, opt_params, gp_params);
metrics.Rob_Ps = zeros(2,2,200); % 2D

% Set initial measurement point.
%points_meas = planning_params.meas_pose_init(1:2);
points_path = [0, 0.5; 0.5, 0; 0.5, 0.5; 0, 0];
points_path_steps = [0; sqrt(sum(diff(points_path,[],1).^2,2))]';
points_path_cumlen = cumsum(points_path_steps);
tq = 0:planning_params.max_vel/planning_params.meas_freq:points_path_cumlen(end);
points_meas = [];
points_meas(:,1) = interp1(points_path_cumlen, points_path(:,1)', tq, 'spline');
points_meas(:,2) = interp1(points_path_cumlen, points_path(:,2)', tq, 'spline');

% Start timer.
time_elapsed = 0;
tic;

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
        Rob_P = [amcl_pose_msg.Pose.Covariance(1), amcl_pose_msg.Pose.Covariance(2); ...
            amcl_pose_msg.Pose.Covariance(7), amcl_pose_msg.Pose.Covariance(8)];
        T_MAP_LINK = trvec2tform([point_current, 0]);
        T_MAP_TEMP = T_MAP_LINK * transforms.T_LINK_TEMP;
        x_MAP_TEMP = tform2trvec(T_MAP_TEMP);
        % Update the GP.
        temp_msg = receive(temp_sub);
        temp = temp_msg.Data;
        
        [field_map, training_data] = ...
            take_measurement_at_point_ros(x_MAP_TEMP(1:2), Rob_P, temp, field_map, ...
            training_data, testing_data, gp_params);
        
        current_time = toc;
        metrics.times = [metrics.times; current_time];
        metrics.points_meas = [metrics.points_meas; point_current];
        metrics.measurements = [metrics.measurements; training_data.Y_train(end)];
        metrics.P_traces = [metrics.P_traces; sum(field_map.cov)];
        metrics.Rob_Ps(:,:,size(metrics.times,1)) = Rob_P;
        
        keyboard
        
    end
    
    %% Planning. -- TO DO.
    %{
    % I. Grid search.
    points_path = search_lattice(Rob, lattice, field_map, ...
        SimLmk, Sen, Lmk, Obs, Trj, Frm, Fac, factorRob, Opt, ...
        training_data, testing_data, map_params, planning_params, gp_params);
    
    % II. Trajectory optimization.
    if (strcmp(opt_params.opt_method, 'cmaes'))
        path_optimized = optimize_with_cmaes(points_path, field_map, ...
            Rob, Sen, SimLmk, Lmk, Obs, Trj, Frm, Fac, factorRob, Opt, ...
            num_control_frames, currentFrame, training_data, testing_data, ...
            map_params, planning_params, opt_params, gp_params);
    else
        path_optimized = points_path;
    end
    
    disp('Next path: ')
    disp(path_optimized)
    disp(['Time: ', num2str(Map.t)])
    
    % Create polynomial path through the control points.
    trajectory = plan_path_waypoints(path_optimized, planning_params.max_vel, ...
        planning_params.max_acc);
    % Sample trajectory for motion simulation.
    [~, points_control, ~, ~] = ...
        sample_trajectory(trajectory, 1/planning_params.control_freq);
    
    metrics.path_travelled = [metrics.path_travelled; path_optimized];
    
    
    % Figure of the Field:
        FldFig = drawFldFig(FldFig,  ...
            Rob, Lmk, ...
            SimRob, ...
            field_map.mean, field_map.cov, ...
            FigOpt);
    
    % Do draw all objects
    drawnow;
    %}
    
end
