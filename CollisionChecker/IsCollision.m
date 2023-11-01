%% IsCollision
% Given a robot model (robot), and trajectory (i.e. joint state vector) (qMatrix)
% and triangle obstacles in the environment (faces,vertex,faceNormals)
function result = IsCollision(robot, qMatrix, cupVertices, returnOnceFound)
if nargin < 4
    returnOnceFound = true;
end
result = false;

for qIndex = 1:size(qMatrix, 1)
    % Get the transform of every joint (i.e. start and end of every link)
    tr = GetLinkPoses(qMatrix(qIndex, :), robot);

    % Go through each link and also each cup vertex
    for i = 1:size(tr, 3) - 1
        for vertexIndex = 1:size(cupVertices, 1)
            vertOnPlane = cupVertices(vertexIndex, :);
            [intersectP, check] = LinePlaneIntersection([0, 0, 1], vertOnPlane, tr(1:3, 4, i)', tr(1:3, 4, i+1)');
            if check == 1 && IsIntersectionPointInsideTriangle(intersectP, cupVertices)
                plot3(intersectP(1), intersectP(2), intersectP(3), 'g*');
                disp('Collision');
                result = true;
                if returnOnceFound
                    return
                end
            end
        end
    end
end
end