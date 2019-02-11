function goal_msg = create_goal_msg(point_current, point_goal, goal_msg)
% Creates pose command message for TurtleBot3,
% facing the direction of motion.

% Set position.
goal_msg.Goal.TargetPose.Pose.Position.X = point_goal(1);
goal_msg.Goal.TargetPose.Pose.Position.Y = point_goal(2);

% Set orientation.
yaw = atan2(point_goal(2) - point_current(2), point_goal(1) - point_current(1));
if (yaw > pi)
    yaw = yaw - 2*pi;
elseif (yaw <= -pi)
    yaw = yaw + 2*pi;
end

q = eul2quat([yaw 0 0]);

goal_msg.Goal.TargetPose.Pose.Orientation.X = q(2);
goal_msg.Goal.TargetPose.Pose.Orientation.Y = q(3);
goal_msg.Goal.TargetPose.Pose.Orientation.Z = q(4);
goal_msg.Goal.TargetPose.Pose.Orientation.W = q(1);

end

