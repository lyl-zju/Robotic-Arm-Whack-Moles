function [qdWaypoints, qddWaypoints] = make_hit_motion_boundaries( ...
    qWaypoints, cfgController, qdStart, hoverWaypointIndex)
if nargin < 3 || isempty(qdStart)
    qdStart = zeros(1, size(qWaypoints, 2));
end
if nargin < 4
    hoverWaypointIndex = [];
end

motionMode = "hover_stop";
if isfield(cfgController, "motion_mode")
    motionMode = normalize_motion_mode(cfgController.motion_mode);
end

qdWaypoints = continuous_velocity_waypoints(qWaypoints, qdStart, cfgController.segment_time);

switch motionMode
    case "hover_stop"
        hoverWaypointIndex = hoverWaypointIndex( ...
            hoverWaypointIndex >= 1 & hoverWaypointIndex <= size(qWaypoints, 1));
        qdWaypoints(hoverWaypointIndex, :) = 0;
    case "continuous"
        % Keep the center-difference velocity at the hover waypoint.
    otherwise
        error("Unknown hit motion mode: %s.", motionMode);
end

qdWaypoints(1, :) = qdStart;
qdWaypoints(end, :) = zeros(1, size(qWaypoints, 2));
qddWaypoints = zeros(size(qWaypoints));
end

function qdWaypoints = continuous_velocity_waypoints(qWaypoints, qdStart, segmentTime)
qdWaypoints = zeros(size(qWaypoints));
qdWaypoints(1, :) = qdStart;

for i = 2:(size(qWaypoints, 1) - 1)
    qdWaypoints(i, :) = (qWaypoints(i + 1, :) - qWaypoints(i - 1, :)) / (2 * segmentTime);
end

maxAbsWaypointSpeed = 1.8;
qdWaypoints = min(max(qdWaypoints, -maxAbsWaypointSpeed), maxAbsWaypointSpeed);
qdWaypoints(1, :) = qdStart;
qdWaypoints(end, :) = zeros(1, size(qWaypoints, 2));
end
