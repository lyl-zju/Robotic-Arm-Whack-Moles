function safe = is_collision_free(jointAngles, robot, cfgRobot, cfgBoard, options)
%IS_COLLISION_FREE Conservative 3-D collision check for joint-space planning.
%
% safe = is_collision_free(q)
% safe = is_collision_free(q, robot, cfgRobot, cfgBoard, options)
%
% options.obstacles supports:
%   sphere:   struct("type","sphere",   "center",[x y z], "radius", r)
%   box:      struct("type","box",      "center",[x y z], "size", [sx sy sz])
%   cylinder: struct("type","cylinder", "center",[x y z], "radius", r, "height", h)
%
% The arm is approximated by sampled capsules along link-frame origins. This
% is intentionally fast and conservative so RRT can call it many times.

if nargin < 2
    robot = [];
end
if nargin < 3
    cfgRobot = [];
end
if nargin < 4
    cfgBoard = [];
end
if nargin < 5
    options = struct();
end
options = fill_collision_defaults(options);

q = jointAngles(:).';
safe = all(isfinite(q));
if ~safe || isempty(robot) || isempty(cfgRobot)
    return;
end

try
    linkPoints = robot_link_points(robot, q, cfgRobot);
catch
    safe = true;
    return;
end

if any(linkPoints(:, 3) < options.min_z)
    safe = false;
    return;
end

samples = sample_link_polyline(linkPoints, options.link_sample_resolution);

if ~isempty(cfgBoard) && options.check_board_box
    boardObstacle = board_as_box_obstacle(cfgBoard, options);
    if points_hit_obstacle(samples, boardObstacle, options.link_radius, options.safety_margin)
        safe = false;
        return;
    end
end

for k = 1:numel(options.obstacles)
    if points_hit_obstacle(samples, options.obstacles(k), options.link_radius, options.safety_margin)
        safe = false;
        return;
    end
end
end

function options = fill_collision_defaults(options)
if ~isfield(options, "min_z") || isempty(options.min_z)
    options.min_z = -0.005;
end
if ~isfield(options, "link_radius") || isempty(options.link_radius)
    options.link_radius = 0.035;
end
if ~isfield(options, "safety_margin") || isempty(options.safety_margin)
    options.safety_margin = 0.015;
end
if ~isfield(options, "link_sample_resolution") || isempty(options.link_sample_resolution)
    options.link_sample_resolution = 0.025;
end
if ~isfield(options, "check_board_box") || isempty(options.check_board_box)
    options.check_board_box = false;
end
if ~isfield(options, "board_half_thickness") || isempty(options.board_half_thickness)
    options.board_half_thickness = 0.025;
end
if ~isfield(options, "obstacles") || isempty(options.obstacles)
    options.obstacles = struct([]);
end
end

function points = robot_link_points(robot, q, cfgRobot)
bodyNames = robot.BodyNames;
points = zeros(numel(bodyNames), 3);
for i = 1:numel(bodyNames)
    tform = getTransform(robot, q, bodyNames{i});
    points(i, :) = tform2trvec(tform);
end

if any(strcmp(bodyNames, cfgRobot.end_effector))
    eeTform = getTransform(robot, q, cfgRobot.end_effector);
    points = [points; tform2trvec(eeTform)]; %#ok<AGROW>
end
end

function samples = sample_link_polyline(points, resolution)
samples = points(1, :);
for i = 1:(size(points, 1) - 1)
    p1 = points(i, :);
    p2 = points(i + 1, :);
    distance = norm(p2 - p1);
    numSteps = max(1, ceil(distance / resolution));
    for k = 1:numSteps
        alpha = k / numSteps;
        samples = [samples; p1 + alpha * (p2 - p1)]; %#ok<AGROW>
    end
end
end

function obstacle = board_as_box_obstacle(cfgBoard, options)
obstacle.type = "box";
obstacle.center = cfgBoard.center_world(:).';
obstacle.size = [cfgBoard.width, cfgBoard.height, 2 * options.board_half_thickness];
obstacle.rotation = cfgBoard.R_world_board;
obstacle.radius = [];
obstacle.height = [];
end

function hit = points_hit_obstacle(points, obstacle, linkRadius, safetyMargin)
padding = linkRadius + safetyMargin;
type = string(obstacle.type);
switch type
    case "sphere"
        center = obstacle.center(:).';
        radius = obstacle.radius + padding;
        hit = any(vecnorm(points - center, 2, 2) <= radius);

    case "box"
        center = obstacle.center(:).';
        halfSize = obstacle.size(:).' / 2 + padding;
        if isfield(obstacle, "rotation") && ~isempty(obstacle.rotation)
            localPoints = (obstacle.rotation.' * (points - center).').';
        else
            localPoints = points - center;
        end
        hit = any(all(abs(localPoints) <= halfSize, 2));

    case "cylinder"
        center = obstacle.center(:).';
        radius = obstacle.radius + padding;
        halfHeight = obstacle.height / 2 + padding;
        if isfield(obstacle, "axis") && ~isempty(obstacle.axis)
            axisDir = obstacle.axis(:) / norm(obstacle.axis);
        else
            axisDir = [0; 0; 1];
        end
        rel = points - center;
        axial = rel * axisDir;
        radialVec = rel - axial * axisDir.';
        hit = any(abs(axial) <= halfHeight & vecnorm(radialVec, 2, 2) <= radius);

    otherwise
        error("Unknown 3-D obstacle type: %s", type);
end
end
