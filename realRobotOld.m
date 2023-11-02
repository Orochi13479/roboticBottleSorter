% To initialise a connection to the ROS computer from Matlab, call rosinit to the
% correct IP address, and then start to subscribe to the joint states
rosshutdown();
rosinit('192.168.27.1'); % If unsure, please ask a tutor
% jointStateSubscriber = rossubscriber('joint_states', 'sensor_msgs/JointState');

%
% pause(2);
%% FIRST
% To get the current joint state from the real robot
jointStateSubscriber = rossubscriber('joint_states', 'sensor_msgs/JointState');
pause(1); % Pause to give time for a message to appear
currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; % Note the default order of the joints is 3,2,1,4,5,6
currentJointState_123456 = [currentJointState_321456(3:-1:1), currentJointState_321456(4:6)];
rad2deg(currentJointState_123456)


% If this fails then try to see what is in the latest message. If it is empty then you are not connected properly.
jointStateSubscriber.LatestMessage

% Before sending commands, we create a variable with the joint names so that the joint commands are associated with a particular joint.
jointNames = {'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};


% In the following example, we are rotating the first (shoulder_pan) and last (wrist_3) joints by pi/8 in 5 seconds.
[client, goal] = rosactionclient('/scaled_pos_joint_traj_controller/follow_joint_trajectory');
goal.Trajectory.JointNames = jointNames;
goal.Trajectory.Header.Seq = 1;
goal.Trajectory.Header.Stamp = rostime('Now', 'system');
goal.GoalTimeTolerance = rosduration(0.05);
bufferSeconds = 1; % This allows for the time taken to send the message. If the network is fast, this could be reduced.
durationSeconds = 5; % This is how many seconds the movement will take

startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
startJointSend.Positions = currentJointState_123456;
startJointSend.TimeFromStart = rosduration(0);

endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
nextJointState_123456 = [pi / 8, -pi / 2, 0, -pi/2, 0, pi / 8];
rad2deg(nextJointState_123456)
endJointSend.Positions = nextJointState_123456;
endJointSend.TimeFromStart = rosduration(durationSeconds);

goal.Trajectory.Points = [startJointSend; endJointSend];
pause(1);
%
goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);

% Then you can send this to the robot
sendGoalAndWait(client, goal,10);

% pause(2);
%% Second
% To get the current joint state from the real robot
disp('here')
jointStateSubscriber = rossubscriber('joint_states', 'sensor_msgs/JointState');
disp('here')
pause(2); % Pause to give time for a message to appear
currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; % Note the default order of the joints is 3,2,1,4,5,6
currentJointState_123456 = [currentJointState_321456(3:-1:1), currentJointState_321456(4:6)];
rad2deg(currentJointState_123456)


% If this fails then try to see what is in the latest message. If it is empty then you are not connected properly.
jointStateSubscriber.LatestMessage

% Before sending commands, we create a variable with the joint names so that the joint commands are associated with a particular joint.
jointNames = {'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};


% In the following example, we are rotating the first (shoulder_pan) and last (wrist_3) joints by pi/8 in 5 seconds.
[client, secondGoal] = rosactionclient('/scaled_pos_joint_traj_controller/follow_joint_trajectory');
secondGoal.Trajectory.JointNames = jointNames;
secondGoal.Trajectory.Header.Seq = 1;
secondGoal.Trajectory.Header.Stamp = rostime('Now', 'system');
secondGoal.GoalTimeTolerance = rosduration(0.05);
bufferSeconds = 1; % This allows for the time taken to send the message. If the network is fast, this could be reduced.
durationSeconds = 5; % This is how many seconds the movement will take

startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
startJointSend.Positions = currentJointState_123456;
startJointSend.TimeFromStart = rosduration(0);

endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
nextJointState_123456 = [0,-0.2985,0.5986,-1.85,-1.5708,0];
rad2deg(nextJointState_123456)
endJointSend.Positions = nextJointState_123456;
endJointSend.TimeFromStart = rosduration(durationSeconds);

secondGoal.Trajectory.Points = [startJointSend; endJointSend];
pause(1);

secondGoal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);

% Then you can send this to the robot
sendGoalAndWait(client, secondGoal,10);

% pause(2);
