function [metrics] = slam_gp_ros(map_params, planning_params, opt_params, gp_params, ...
    training_data, gt_data, testing_data)
% TurtleBot3 temperature mapping experiments for RAL-19.
% ---
% M Popovic 2019
%

%clear

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
lattice = create_lattice(map_params, planning_params);
% GP field map
field_map = [];

% Initialise data logging.
metrics = initialize_metrics(map_params, planning_params, opt_params, gp_params);

% Go to initial measurement point.
reached_point = false;
point_init = planning_params.meas_pose_init(1:2);
while (~reached_point)
    amcl_pose_msg = receive(amcl_pose_sub);
    point_current = [amcl_pose_msg.Pose.Pose.Position.X, ...
        amcl_pose_msg.Pose.Pose.Position.Y];
    
    if (pdist2(point_init, point_current) < planning_params.achievement_dist)
        reached_point = true;
    end
end

time_elapsed = 0;

points_goal = point_init;

%% V. Main loop
while (true)
    
    % Check if planning budget exceeded
    if (time_elapsed > planning_params.time_budget)
        disp('Time budget exceeded!')
        return;
    end
    
    %% Sensing.
    % Take measurements along path, updating the GP field map.
    for i = 1:size(points_goal)
        
        % Send the command.
        amcl_pose_msg = receive(amcl_pose_sub);
        point_current = [amcl_pose_msg.Pose.Pose.Position.X, ...
            amcl_pose_msg.Pose.Pose.Position.Y];
        goal_msg = create_goal_msg(point_current, points_goal(i,:), goal_msg);
        send(goal_pub, goal_msg);
        
        % Wait to reach target measurement point.
        reached_point = false;
        while (~reached_point)
            amcl_pose_msg = receive(amcl_pose_sub);
            point_current = [amcl_pose_msg.Pose.Pose.Position.X, ...
                amcl_pose_msg.Pose.Pose.Position.Y];
            if (pdist2(points_goal(i,:), point_current) < planning_params.achievement_dist)
                reached_point = true;
            end
        end
        
        % Update the GP.
        [field_map, training_data] = ...
            take_measurement_at_point(Rob(rob).state.x(1:3)', ...
            SimRob(rob).state.x(1:3)', Rob_P, field_map, .....
            training_data, gt_data, testing_data, gp_params, map_params);
        
        metrics.times = [metrics.times; Map.t];
        metrics.points_meas = [metrics.points_meas; Rob(rob).state.x(1:3)'];
        metrics.points_meas_gt = [metrics.points_meas_gt; SimRob(rob).state.x(1:3)'];
        metrics.measurements = [metrics.measurements; training_data.Y_train(end)];
        metrics.P_traces = [metrics.P_traces; sum(field_map.cov)];
        metrics.rmses = [metrics.rmses; compute_rmse(field_map.mean, gt_data.Y_gt)];
        metrics.mlls = [metrics.mlls; compute_mll(field_map, gt_data.Y_gt)];
        metrics.Rob_Ps(:,:,size(metrics.times,1)) = Rob_P;
        
        %disp(['Distance between real + estimated robot positions: ', ...
        %    num2str(pdist([Rob.state.x(1:3)'; SimRob.state.x(1:3)']))])
        disp(['Map RMSE = ', num2str(metrics.rmses(end))])
        
        
    end
    
    %% Planning.
    
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
    if (refresh_field_fig)
        FldFig = drawFldFig(FldFig,  ...
            Rob, Lmk, ...
            SimRob, ...
            field_map.mean, field_map.cov, ...
            FigOpt);
        refresh_field_fig = 0;
    end
    
    % Do draw all objects
    drawnow;

    
end
