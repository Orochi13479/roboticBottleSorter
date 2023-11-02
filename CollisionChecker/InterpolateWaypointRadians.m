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