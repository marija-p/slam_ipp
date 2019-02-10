close all

points_meas = metrics.points_meas;
points_meas_gt = metrics.points_meas_gt;

figure;
hold on
trajectory_meas = ...
            plan_path_waypoints(points_meas, ...
            planning_params.max_vel, ...
            planning_params.max_acc);
[t_meas, p_meas] = sample_trajectory(trajectory_meas, 0.1);
cline(p_meas(:,1), p_meas(:,2), p_meas(:,3), t_meas);

%trajectory_meas_gt = ...
%            plan_path_waypoints(points_meas_gt, ...
%            planning_params.max_vel, ...
%            planning_params.max_acc);
%[t_meas_gt, p_meas_gt] = sample_trajectory(trajectory_meas_gt, 0.1);
%cline(p_meas_gt(:,1), p_meas_gt(:,2), p_meas_gt(:,3), t_meas_gt);

colors_meas = linspace(0, t_meas(end),size(points_meas,1));
scatter3(points_meas(:,1), points_meas(:,2), points_meas(:,3), 60, colors_meas, 'filled');

plot(metrics.landmarks(1,:), metrics.landmarks(2,:), 'sk', 'MarkerSize', 10)

view(3);
pbaspect([1 1 1])
grid on
grid minor
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
axis([-2.5 2.5 -2.5 2.5 0 5])
colormap jet

set(findall(gcf,'-property','FontName'),'FontName','Times')
