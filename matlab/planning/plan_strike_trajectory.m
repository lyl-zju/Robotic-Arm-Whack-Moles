function [traj, planInfo] = plan_strike_trajectory(robot, currentJoints, targetCartesianPose, cfgRobot, cfgBoard, cfgController, options)
%PLAN_STRIKE_TRAJECTORY Seeded IK + joint-space RRT-Connect strike planner.
%
% [traj, planInfo] = plan_strike_trajectory(robot, currentJoints, targetPose,
%     cfgRobot, cfgBoard, cfgController, options)
%
% 该入口面向 MATLAB 主仿真：先用当前关节角作为 IK seed 求目标上方 hover
% 姿态，再在关节空间做 RRT-Connect，平滑后追加 hit/back 关键点并生成
% plan_joint_traj 可执行轨迹。

if nargin < 7
    options = struct();
end
options = fill_strike_defaults(options, currentJoints);

currentJoints = currentJoints(:).';
targetWorld = extract_target_world(targetCartesianPose);
keyposes = make_hit_keyposes(targetWorld, cfgBoard);

% 使用当前关节角作为 IK 初值，压住肘部翻转和腕部绕圈。
qHover = solve_ik(robot, keyposes.hover, cfgRobot, currentJoints);
qHover = wrap_to_seed(qHover, currentJoints);

qHit = solve_ik(robot, keyposes.hit, cfgRobot, qHover);
qHit = wrap_to_seed(qHit, qHover);

qBack = solve_ik(robot, keyposes.back, cfgRobot, qHit);
qBack = wrap_to_seed(qBack, qHit);

if ~within_limits(qHover, options.joint_limits)
    error("Hover IK solution is outside joint limits.");
end
if ~within_limits(qHit, options.joint_limits)
    error("Hit IK solution is outside joint limits.");
end
if ~within_limits(qBack, options.joint_limits)
    error("Back IK solution is outside joint limits.");
end

collisionOptions = options.collision_options;
baseChecker = @(q)is_collision_free(q, robot, cfgRobot, cfgBoard, collisionOptions);
if ~isempty(options.collision_checker)
    userChecker = options.collision_checker;
    checker = @(q)baseChecker(q) && userChecker(q);
else
    checker = baseChecker;
end

rrtOptions.goal_bias = options.goal_bias;
rrtOptions.collision_resolution = options.collision_resolution;
rrtOptions.collision_checker = checker;
if isfield(options, "seed") && ~isempty(options.seed)
    rrtOptions.seed = options.seed;
end

[rawPath, rrtInfo] = plan_joint_path( ...
    currentJoints, qHover, options.joint_limits, ...
    options.max_iter, options.step_size, rrtOptions);

if isempty(rawPath) || ~rrtInfo.found
    error("Joint-space RRT-Connect failed to reach the hover pose.");
end

smoothOptions.collision_checker = checker;
smoothOptions.collision_resolution = options.collision_resolution;
smoothOptions.interpolation_step = options.interpolation_step;
smoothOptions.method = options.interpolation_method;
[smoothPath, smoothInfo] = smooth_trajectory(rawPath, options.max_smooth_iterations, smoothOptions);

% 去掉 smoothPath 中最后的 hover 点，再追加 hover/hit/back，避免重复点。
qWaypoints = smoothPath;
if norm(qWaypoints(end, :) - qHover) > 1e-9
    qWaypoints = [qWaypoints; qHover]; %#ok<AGROW>
end
qWaypoints = [qWaypoints; qHit; qBack];

hoverWaypointIndex = size(qWaypoints, 1) - 2;
if isfield(options, "start_velocity") && ~isempty(options.start_velocity)
    qdStart = options.start_velocity(:).';
else
    qdStart = zeros(size(currentJoints));
end
[qdWaypoints, qddWaypoints] = make_hit_motion_boundaries( ...
    qWaypoints, cfgController, qdStart, hoverWaypointIndex);

traj = plan_joint_traj(qWaypoints, cfgController.segment_time, cfgController.dt, ...
    qdWaypoints, qddWaypoints);
traj.path_type = "joint_rrt_connect";
traj.motion_mode = cfgController.motion_mode;
traj.q_waypoints = qWaypoints;
traj.rrt_raw_path = rawPath;
traj.rrt_smooth_path = smoothPath;

planInfo.target_world = targetWorld;
planInfo.keyposes = keyposes;
planInfo.q_hover = qHover;
planInfo.q_hit = qHit;
planInfo.q_back = qBack;
planInfo.raw_path = rawPath;
planInfo.smooth_path = smoothPath;
planInfo.rrt_info = rrtInfo;
planInfo.smooth_info = smoothInfo;
planInfo.q_waypoints = qWaypoints;
end

function options = fill_strike_defaults(options, currentJoints)
numJoints = numel(currentJoints);
if ~isfield(options, "joint_limits") || isempty(options.joint_limits)
    options.joint_limits = repmat([-pi, pi], numJoints, 1);
end
if ~isfield(options, "max_iter") || isempty(options.max_iter)
    options.max_iter = 1000;
end
if ~isfield(options, "step_size") || isempty(options.step_size)
    options.step_size = 0.1;
end
if ~isfield(options, "goal_bias") || isempty(options.goal_bias)
    options.goal_bias = 0.15;
end
if ~isfield(options, "collision_resolution") || isempty(options.collision_resolution)
    options.collision_resolution = max(options.step_size / 2, 0.02);
end
if ~isfield(options, "max_smooth_iterations") || isempty(options.max_smooth_iterations)
    options.max_smooth_iterations = 50;
end
if ~isfield(options, "interpolation_step") || isempty(options.interpolation_step)
    options.interpolation_step = 0.04;
end
if ~isfield(options, "interpolation_method") || isempty(options.interpolation_method)
    options.interpolation_method = "pchip";
else
    options.interpolation_method = string(options.interpolation_method);
end
if ~isfield(options, "collision_options") || isempty(options.collision_options)
    options.collision_options = struct();
end
if ~isfield(options, "collision_checker")
    options.collision_checker = [];
end
options.joint_limits = validate_limits(options.joint_limits, numJoints);
end

function limits = validate_limits(limits, numJoints)
limits = double(limits);
if ~isequal(size(limits), [numJoints, 2])
    error("options.joint_limits must have size numJoints x 2.");
end
if any(limits(:, 1) >= limits(:, 2))
    error("Each joint limit must satisfy lower < upper.");
end
end

function targetWorld = extract_target_world(targetPose)
if isstruct(targetPose)
    if isfield(targetPose, "position_world")
        targetWorld = targetPose.position_world;
    elseif isfield(targetPose, "world")
        targetWorld = targetPose.world;
    elseif isfield(targetPose, "position")
        targetWorld = targetPose.position;
    else
        error("targetCartesianPose struct must contain position_world, world, or position.");
    end
else
    targetWorld = targetPose;
end
targetWorld = targetWorld(:).';
if numel(targetWorld) ~= 3
    error("targetCartesianPose must resolve to a 1x3 world position.");
end
end

function qWrapped = wrap_to_seed(q, seed)
q = q(:).';
seed = seed(:).';
qWrapped = seed + atan2(sin(q - seed), cos(q - seed));
end

function ok = within_limits(q, limits)
q = q(:).';
ok = all(q >= limits(:, 1).' - 1e-10) && all(q <= limits(:, 2).' + 1e-10);
end
