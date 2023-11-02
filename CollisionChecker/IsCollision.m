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