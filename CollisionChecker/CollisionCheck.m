%% Collision Checker and Avoidance Function
function CollisionCheck(robot, cupVertices)
% Creating transform for every joint
q = robot.getpos();
tr = zeros(4, 4, robot.n+1);
tr(:, :, 1) = robot.base;
L = robot.links;
for i = 1:robot.n
    tr(:, :, i+1) = tr(:, :, i) * trotz(q(i)+L(i).offset) * transl(0, 0, L(i).d) * transl(L(i).a, 0, 0) * trotx(L(i).alpha);
end

for i = 1:size(tr, 3) - 1
    for vertexIndex = 1:size(cupVertices, 1)
        vertOnPlane = cupVertices(vertexIndex, :);
        [intersectP, check] = LinePlaneIntersection([0, 0, 1], vertOnPlane, tr(1:3, 4, i)', tr(1:3, 4, i+1)');
        if check == 1 && IsIntersectionPointInsideTriangle(intersectP, cupVertices)
            plot3(intersectP(1), intersectP(2), intersectP(3), 'g*');
            disp('Collision');
        end
    end
end

% Go through until there are no step sizes larger than 1 degree
q1 = robot.getpos();
q2 = deg2rad([81.5, -12, 50, 51, 90, 0]);

steps = 2;

while ~isempty(find(1 < abs(diff(rad2deg(jtraj(q1, q2, steps)))),1))
    steps = steps + 1;
end
qMatrix = jtraj(q1, q2, steps);

% Check each of the joint states in the trajectory to work out which ones are in collision.
% Return a logical vector of size steps which contains 0 = no collision (safe)
% and 1 = yes collision (Unsafe).

result = true(steps, 1);
for i = 1:steps
    % fprintf("Step: %d\n", i)
    result(i) = IsCollision(robot, qMatrix(i, :), cupVertices, false);
    % robot.animate(qMatrix(i,:));
    % drawnow();
    pause(0.02);
    if result(i) == true
        disp('UNSAFE: Object detected. Robot stopped')
        break
    end
end
end