% Helper function to move the ply vertices
function movePLY(robot, plyArray, verts, posArray, offset)
    endEffector = robot.fkine(robot.getpos()).T;
    endEffector(3, 4) = endEffector(3, 4) + offset;

    % Rotate by 180 degrees around the X-axis to counteract flipping
    R = trotx(pi); % Rotate by 180 degrees around X-axis

    % Apply the rotation to the vertices
    verticesHomogeneous = [verts, ones(size(verts, 1), 1)];
    newTr = (endEffector * R * verticesHomogeneous')';

    % Update the ply object vertices with the corrected transformation
    set(plyArray(posArray), 'Vertices', newTr(:, 1:3));
end
