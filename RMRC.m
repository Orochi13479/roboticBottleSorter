%% FROM Lab9Solution_Question1.m Modified

function RMRC(robot, startTr, endTr, time, deltaT)

steps = time / deltaT; % No. of steps for simulation
delta = 2 * pi / steps; % Small angle change
epsilon = 0.1; % Threshold value for manipulability/Damped Least Squares
W = diag([1, 1, 1, 0.1, 0.1, 0.1]); % Weighting matrix for the velocity vector

% Allocate array data
m = zeros(steps, 1); % Array for Measure of Manipulability
qMatrix = zeros(steps, robot.n); % Array for joint anglesR
qdot = zeros(steps, robot.n); % Array for joint velocities
theta = zeros(3, steps); % Array for roll-pitch-yaw angles

% Set up trajectory
s = lspb(0, 1, steps); % lspb - Linear segment with parabolic bends
q0 = zeros(1, robot.n); % Initial joint angles

for i = 1:steps
    x(1, i) = (1 - s(i)) * 0.35 + s(i) * 0.35; % Points in x
    x(2, i) = (1 - s(i)) * -0.55 + s(i) * 0.55; % Points in y
    x(3, i) = 0.5 + 0.2 * sin(i*delta); % Points in z
    theta(1, i) = 0; % Roll angle
    theta(2, i) = 5 * pi / 9; % Pitch angle
    theta(3, i) = 0; % Yaw angle
end

T = [rpy2r(theta(1, 1), theta(2, 1), theta(3, 1)), x(:, 1); zeros(1, 3), 1]; % Create transformation of first point and angle
qMatrix(1, :) = robot.ikcon(T, q0); % Solve joint angles to achieve first waypoint (this might or might not have some error)

% Track the trajectory with RMRC (error = desired - actual)
for i = 1:steps - 1
    % Solve for the desired linear and angular velocities
    T = robot.fkine(qMatrix(i, :)).T; % Get forward transformation at current joint state (as the joint angles obtained from ikcon are not always accurate)
    deltaX = x(:, i+1) - T(1:3, 4); % x are the position sets and T represents the actual position that the robot is in
    Rd = rpy2r(theta(1, i+1), theta(2, i+1), theta(3, i+1)); % Get next RPY angles, convert to rotation matrix
    Ra = T(1:3, 1:3); % Current end-effector rotation matrix
    Rdot = (1 / deltaT) * (Rd - Ra); % Calculate rotation matrix error
    S = Rdot * Ra'; % Skew symmetric of roll, pitch, and yaw velocities. (Used as the derivative of the rotation matrix is equal to the negative derivative of the transpose of the rotation matrix)
    linear_velocity = (1 / deltaT) * deltaX;
    angular_velocity = [S(3, 2); S(1, 3); S(2, 1)]; % Check the structure of Skew Symmetric matrix!!
    deltaTheta = tr2rpy(abs(Rd*Ra')); % Convert rotation matrix to RPY angles
    xdot = W * [linear_velocity; angular_velocity]; % Calculate end-effector velocity to reach next waypoint.

    % Solve for the Jacobian
    J = robot.jacob0(qMatrix(i, :)); % Get Jacobian at current joint state
    m(i) = sqrt(det(J*J'));
    if m(i) < epsilon % If manipulability is less than given threshold
        lambda = (1 - m(i)/epsilon) * 5E-2;
    else
        lambda = 0;
    end

    invJ = inv(J'*J+lambda*eye(robot.n)) * J'; % Pseudo Inverse of the Jacobian DLS
    qdot(i, :) = (invJ * xdot)'; % Inverse kinematics of velocities (RMRC equation)                                           % Solve the RMRC equation (you may need to transpose the vector)

    for j = 1:robot.n
        if qMatrix(i, j) + deltaT * qdot(i, j) < robot.qlim(j, 1) % If next joint angle is lower than joint limit...
            qdot(i, j) = 0; % Stop the motor
        elseif qMatrix(i, j) + deltaT * qdot(i, j) > robot.qlim(j, 2) % If next joint angle is greater than joint limit ...
            qdot(i, j) = 0; % Stop the motor
        end
    end

    qMatrix(i+1, :) = qMatrix(i, :) + deltaT * qdot(i, :); % Update next joint state based on joint velocities
    updatedQ = qMatrix(i+1, :);
    % Animate the robot with RMRC Traj
    robot.animate(updatedQ)
    drawnow();
end
end