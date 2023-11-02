function [qMatrix] = collisionFreeTraj(robot, q1, q2, vertex, faces)
    
    faceNormals = zeros(size(faces,1),3);
    for faceIndex = 1:size(faces,1)
        v1 = vertex(faces(faceIndex,1)',:);
        v2 = vertex(faces(faceIndex,2)',:);
        v3 = vertex(faces(faceIndex,3)',:);
        faceNormals(faceIndex,:) = unit(cross(v2-v1,v3-v1));
    end
    
    disp("RRT Collision detection")
    
    qWaypoints = [q1; q2];
    isCollision = true;
    checkedTillWaypoint = 1;
    qMatrix = [];
    qLengthOpt = 150; % Upper limit on the acceptable length of RRT Traj, Basically a minimum cost function, Increase if it is taking too long to find a traj
    tic
    while (isCollision)
        startWaypoint = checkedTillWaypoint;
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
end








