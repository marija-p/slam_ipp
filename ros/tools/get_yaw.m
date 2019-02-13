function yaw = get_yaw(point_current, point_goal)
% Gets yaw angle in the direction of motion between two points.

yaw = atan2(point_goal(2) - point_current(2), point_goal(1) - point_current(1));
if (yaw > pi)
    yaw = yaw - 2*pi;
elseif (yaw <= -pi)
    yaw = yaw + 2*pi;
end

end

