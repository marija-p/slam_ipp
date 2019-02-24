function [] = plot_path(metrics, planning_params)
% Visualizes planned trajectory 

figure;

t = [];
p = [];
p_control = metrics.path_travelled;
p_meas = metrics.points_meas;

% Many polynomials.
if isfield(planning_params, 'control_points')
    
    % Loop through all polynomial trajectories that were executed,
    % and stack times/measurements for plotting.
    for i = 1:planning_params.control_points:size(p_control,1)
        
        % Create the (semi-global) trajectory.
        trajectory = ...
            plan_path_waypoints(p_control(i:i+planning_params.control_points-1,:), ...
            planning_params.max_vel, ...
            planning_params.max_acc);
        [t_poly, p_poly] = sample_trajectory(trajectory, 0.1);
        
        if (i == 1)
            t = [t; t_poly'];
        else
            disp(['Trajectory start: ', num2str(t(end))])
            disp(['Trajectory length: ', ...
                num2str(get_trajectory_total_time(trajectory)), 's']);
            t = [t; t(end) + t_poly'];
        end
        p = [p; p_poly];
        
    end
    
% Only one polynomial.
else
    
    % Create the (global) trajectory.
    trajectory = plan_path_waypoints(p_control, planning_params.max_vel, ...
        planning_params.max_acc);
    [t, p] = sample_trajectory(trajectory, 0.1);
    
end

hold on
% Visualize trajectory.
cline(p(:,1), p(:,2), p(:,3), t);
% Visualize control points.
%scatter3(p_control(:,1), p_control(:,2), p_control(:,3), 140, 'xk');
% Visualize measurements.
colors_meas = linspace(0, t(end),size(p_meas,1));

% Silly bug with 3 points.
% https://ch.mathworks.com/matlabcentral/newsreader/view_thread/136731
if isequal(size(colors_meas),[1 3])
    colors_meas = colors_meas';
end

scatter3(p_meas(:,1), p_meas(:,2), p_meas(:,3), 20, colors_meas, 'filled');

xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
axis([-6 6 -6 6 0 7])
grid minor
colormap jet
c = colorbar;
ylabel(c, 'Time (s)')
view(3)
legend('Path', 'Control pts.', 'Meas. pts.', 'Location', 'northeast')

end

