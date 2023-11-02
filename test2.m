%% 3.3: Randomly select waypoints (primative RRT)
close all;
clear all;
clc;

centerpnt = [1.1, 0, 0];
side = 1.5;
plotOptions.plotFaces = true;
folderName = 'data';
[cup, TRI,PTS,DATA] = PlaceObject(fullfile(folderName, 'brownTable.ply'),[0.25,0,0])
% [TRI,PTS,DATA,~] = plyread(fullfile(folderName, 'plasticCup.ply'), 'tri');
vertex = PTS;
faces = TRI;
% faces3 = DATA.face.vertex_indices;
% faceNormals = [DATA.vertex.nx,DATA.vertex.ny,DATA.vertex.nz];
for i=1:length(vertex)
    faceNormal(i) = cross((vertex(1,:)-vertex(2,:)),(vertex(2,i)-vertex(3,:)));
    faceNormals = faceNormal(i) / norm(faceNormal(i));
end
X250Robot = WidowX250;
robot = X250Robot.model();
[armRotationMatrix1, armTranslationVector1] = tr2rt(robot.base);
translationVector1 = [-0.3, -0.6, 0.5];
robot.base = rt2tr(armRotationMatrix1, translationVector1);

hold on
% axis equal
camlight
zlim([-0.05, 2]);
xlim([-2, 2]); % xlim([-4.2, 4.2]); <-- SHOULD PROBS MAKE SMALLER
ylim([-2, 2]); % ylim([-2.5, 2.5]); <-- SHOULD PROBS MAKE SMALLER
robot.delay = 0;
disp("RRT Collision detection")
q1 = [-pi / 4, 0, 0, 0, 0, 0];
q2 = [pi / 4, 0, 0, 0, 0, 0];
robot.animate(q1);
qWaypoints = [q1; q2];
isCollision = true;
checkedTillWaypoint = 1;
qMatrix = [];
qLengthOpt = 300; % Upper limit on the acceptable length of RRT Traj, Basically a minimum cost function, Increase if it is taking too long to find a traj
disp("here")
tic
while (isCollision)
    startWaypoint = checkedTillWaypoint;
    disp("Cycle")
    for i = startWaypoint:size(qWaypoints, 1) - 1
        qMatrixJoin = InterpolateWaypointRadians(qWaypoints(i:i+1, :), deg2rad(10));
        if ~IsCollision(robot, qMatrixJoin, faces, vertex, faceNormals)
            qMatrix = [qMatrix; qMatrixJoin]; %#ok<AGROW>
            isCollision = false;
            checkedTillWaypoint = i + 1;
            % Now try and join to the final goal (q2)
            qMatrixJoin = InterpolateWaypointRadians([qMatrix(end, :); q2], deg2rad(10));
            if ~IsCollision(robot, qMatrixJoin, faces, vertex, faceNormals)
                qMatrix = [qMatrix; qMatrixJoin];
                % Reached goal without collision, so break out
                disp("Found Collision Free Traj")
                break;
            end
        else
            % Randomly pick a pose that is not in collision
            qRand = (2 * rand(1, 6) - 1) * pi;
            while IsCollision(robot, qRand, faces, vertex, faceNormals)
                qRand = (2 * rand(1, 6) - 1) * pi;
            end
            qWaypoints = [qWaypoints(1:i, :); qRand; qWaypoints(i+1:end, :)];
            isCollision = true;
            break;
        end
    end

    if length(qMatrix) > qLengthOpt
        disp("Traj too Long Trying again")
        % Reset Variables to rerun RRT
        qWaypoints = [q1; q2];
        isCollision = true;
        checkedTillWaypoint = 1;
        qMatrix = [];
        continue
    end
end
disp("RRT Took: "+num2str(toc));
disp("qMatrix Length: "+num2str(length(qMatrix)));
pause(5)
robot.animate(qMatrix)
drawnow();
%% IsIntersectionPointInsideTriangle
% Given a point which is known to be on the same plane as the triangle
% determine if the point is
% inside (result == 1) or
% outside a triangle (result ==0 )
function result = IsIntersectionPointInsideTriangle(intersectP, triangleVerts)

u = triangleVerts(2, :) - triangleVerts(1, :);
v = triangleVerts(3, :) - triangleVerts(1, :);

uu = dot(u, u);
uv = dot(u, v);
vv = dot(v, v);

w = intersectP - triangleVerts(1, :);
wu = dot(w, u);
wv = dot(w, v);

D = uv * uv - uu * vv;

% Get and test parametric coords (s and t)
s = (uv * wv - vv * wu) / D;
if (s < 0.0 || s > 1.0) % intersectP is outside Triangle
    result = 0;
    return;
end

t = (uv * wu - uu * wv) / D;
if (t < 0.0 || (s + t) > 1.0) % intersectP is outside Triangle
    result = 0;
    return;
end

result = 1; % intersectP is in Triangle
end

%% IsCollision
% This is based upon the output of questions 2.5 and 2.6
% Given a robot model (robot), and trajectory (i.e. joint state vector) (qMatrix)
% and triangle obstacles in the environment (faces,vertex,faceNormals)
function result = IsCollision(robot, qMatrix, faces, vertex, faceNormals, returnOnceFound)
if nargin < 6
    returnOnceFound = true;
end
result = false;

for qIndex = 1:size(qMatrix, 1)
    % Get the transform of every joint (i.e. start and end of every link)
    tr = GetLinkPoses(qMatrix(qIndex, :), robot);

    % Go through each link and also each triangle face
    for i = 1:size(tr, 3) - 1
        for faceIndex = 1:size(faces, 1)
            vertOnPlane = vertex(faces(faceIndex, 1)', :);
            [intersectP, check] = LinePlaneIntersection(faceNormals(faceIndex, :), vertOnPlane, tr(1:3, 4, i)', tr(1:3, 4, i+1)');
            if check == 1 && IsIntersectionPointInsideTriangle(intersectP, vertex(faces(faceIndex, :)', :))
                plot3(intersectP(1), intersectP(2), intersectP(3), 'g*');
                % disp('Intersection');
                result = true;
                if returnOnceFound
                    return
                end
            end
        end
    end
end
end

%% GetLinkPoses
% q - robot joint angles
% robot -  seriallink robot model
% transforms - list of transforms
function [transforms] = GetLinkPoses(q, robot)

links = robot.links;
transforms = zeros(4, 4, length(links)+1);
transforms(:, :, 1) = robot.base;

for i = 1:length(links)
    L = links(1, i);

    current_transform = transforms(:, :, i);

    current_transform = current_transform * trotz(q(1, i)+L.offset) * ...
        transl(0, 0, L.d) * transl(L.a, 0, 0) * trotx(L.alpha);
    transforms(:, :, i+1) = current_transform;
end
end

%% FineInterpolation
% Use results from Q2.6 to keep calling jtraj until all step sizes are
% smaller than a given max steps size
function qMatrix = FineInterpolation(q1, q2, maxStepRadians)
if nargin < 3
    maxStepRadians = deg2rad(1);
end

steps = 2;
while ~isempty(find(maxStepRadians < abs(diff(jtraj(q1, q2, steps))), 1))
    steps = steps + 1;
end
qMatrix = jtraj(q1, q2, steps);
end

%% InterpolateWaypointRadians
% Given a set of waypoints, finely intepolate them
function qMatrix = InterpolateWaypointRadians(waypointRadians, maxStepRadians)
if nargin < 2
    maxStepRadians = deg2rad(1);
end

qMatrix = [];
for i = 1:size(waypointRadians, 1) - 1
    qMatrix = [qMatrix; FineInterpolation(waypointRadians(i, :), waypointRadians(i+1, :), maxStepRadians)]; %#ok<AGROW>
end
end