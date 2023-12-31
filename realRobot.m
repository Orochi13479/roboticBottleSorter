% To initialise a connection to the ROS computer from Matlab, call rosinit to the
% correct IP address, and then start to subscribe to the joint states
rosshutdown();
rosinit('192.168.27.1'); % Replace this with the appropriate IP address

%% Define the joint names
jointNames = {'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};

% Define the target joint states for each step
targetJointStates = [
    [pi / 8, -pi / 2, 0, -pi / 2, 0, pi / 8];
    deg2rad([170,-45,66,-110,-90,22.5]);
    deg2rad([280, -66, 86, -110, -90, 22.5]);
    deg2rad([350, -30, 25, -86, -90, 22.5]);
    deg2rad([120,-35.5, 45.5,-100,-90,22.5]);
    deg2rad([280, -66, 86, -110, -90, 22.5]);
    deg2rad([350, -30, 25, -86, -90, 22.5]);
    [pi / 8, -pi / 2, 0, -pi / 2, 0, pi / 8]
] ;

% Define the action client and goal
[client, goal] = rosactionclient('/scaled_pos_joint_traj_controller/follow_joint_trajectory');
goal.Trajectory.JointNames = jointNames;
goal.GoalTimeTolerance = rosduration(0.05);
bufferSeconds = 1; % Buffer time to account for message sending time

for i = 1:size(targetJointStates, 1)
    % Get the current joint state
    jointStateSubscriber = rossubscriber('joint_states', 'sensor_msgs/JointState');
    pause(2);
    currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)';
    currentJointState_123456 = [currentJointState_321456(3:-1:1), currentJointState_321456(4:6)];

    % Define the start and end joint states for this action step
    startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
    startJointSend.Positions = currentJointState_123456;
    startJointSend.TimeFromStart = rosduration(0);
    
    endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
    endJointSend.Positions = targetJointStates(i, :);
    endJointSend.TimeFromStart = rosduration(5);
    
    goal.Trajectory.Points = [startJointSend; endJointSend];
    
    % Set the goal header
    goal.Trajectory.Header.Seq = i;

    goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
        
    % Send the goal to the robot Wait for the action to complete
    sendGoalAndWait(client, goal, 10)
end

% Cleanup ROS nodes
% rosshutdown();
