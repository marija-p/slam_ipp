goal_pub = rospublisher('/move_base/goal');
goal_msg = rosmessage(goal_pub);
pose_sub = rossubscriber('/amcl_pose');

goal_pos = [0,0];

pose_msg = receive(pose_sub);
current_pos = [pose_msg.Pose.Pose.Position.X, pose_msg.Pose.Pose.Position.Y];

goal_msg.Goal.TargetPose.Header.FrameId = 'map';

% Set position.
goal_msg.Goal.TargetPose.Pose.Position.X = goal_pos(1);
goal_msg.Goal.TargetPose.Pose.Position.Y = goal_pos(2);
% Set orientation.
yaw = atan2(goal_pos(2) - current_pos(2), goal_pos(1) - current_pos(1));
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


send(goal_pub, goal_msg)

