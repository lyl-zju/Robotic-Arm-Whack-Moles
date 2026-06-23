function [path, info] = plan_joint_path(startJoints, goalJoints, jointLimits, maxIter, stepSize, options)
%PLAN_JOINT_PATH Joint-space bidirectional RRT-Connect planner.
%
% path = plan_joint_path(startJoints, goalJoints, jointLimits)
% [path, info] = plan_joint_path(..., maxIter, stepSize, options)
%
% 输入:
%   startJoints : 1xn 当前关节角
%   goalJoints  : 1xn 目标关节角
%   jointLimits : nx2 关节限位 [lower upper]
%   maxIter     : 最大采样次数，默认 1000
%   stepSize    : 每次扩展的关节空间步长，默认 0.1 rad
%   options     : 可选结构体
%       goal_bias            目标偏置概率，默认 0.15
%       collision_resolution 边检查分辨率，默认 stepSize/2
%       collision_checker    函数句柄 @(q)isSafe
%       seed                 随机种子
%
% 输出:
%   path : kxn 关节空间无碰撞路径
%   info : 调试信息

if nargin < 4 || isempty(maxIter)
    maxIter = 1000;
end
if nargin < 5 || isempty(stepSize)
    stepSize = 0.1;
end
if nargin < 6
    options = struct();
end
options = fill_rrt_connect_defaults(options, stepSize);

if isfield(options, "seed") && ~isempty(options.seed)
    rng(options.seed);
end

startJoints = startJoints(:).';
goalJoints = goalJoints(:).';
jointLimits = validate_joint_limits(jointLimits, numel(startJoints));

if numel(goalJoints) ~= numel(startJoints)
    error("startJoints and goalJoints must have the same length.");
end
if stepSize <= 0
    error("stepSize must be positive.");
end
if maxIter <= 0
    error("maxIter must be positive.");
end

checker = options.collision_checker;
if ~within_joint_limits(startJoints, jointLimits)
    error("startJoints are outside joint limits.");
end
if ~within_joint_limits(goalJoints, jointLimits)
    error("goalJoints are outside joint limits.");
end
if ~checker(startJoints)
    error("startJoints are in collision.");
end
if ~checker(goalJoints)
    error("goalJoints are in collision.");
end

if norm(goalJoints - startJoints) <= stepSize && ...
        edge_collision_free(startJoints, goalJoints, options.collision_resolution, checker)
    path = deduplicate_path([startJoints; goalJoints]);
    info = make_info(true, 0, path, [0; 1], path, [0; 1]);
    return;
end

treeA.nodes = startJoints;
treeA.parents = 0;
treeB.nodes = goalJoints;
treeB.parents = 0;

found = false;
connectA = NaN;
connectB = NaN;
activeWasStart = true;

for iter = 1:maxIter
    if size(treeA.nodes, 1) <= size(treeB.nodes, 1)
        activeName = "A";
        activeWasStart = true;
    else
        activeName = "B";
        activeWasStart = false;
    end

    if rand < options.goal_bias
        if activeWasStart
            sample = goalJoints;
        else
            sample = startJoints;
        end
    else
        sample = sample_joint_limits(jointLimits);
    end

    if activeName == "A"
        [treeA, newIdx] = extend_tree(treeA, sample, stepSize, jointLimits, options.collision_resolution, checker);
        if isnan(newIdx)
            continue;
        end
        [treeB, otherIdx] = connect_tree(treeB, treeA.nodes(newIdx, :), stepSize, jointLimits, options.collision_resolution, checker);
        if ~isnan(otherIdx)
            found = true;
            connectA = newIdx;
            connectB = otherIdx;
            break;
        end
    else
        [treeB, newIdx] = extend_tree(treeB, sample, stepSize, jointLimits, options.collision_resolution, checker);
        if isnan(newIdx)
            continue;
        end
        [treeA, otherIdx] = connect_tree(treeA, treeB.nodes(newIdx, :), stepSize, jointLimits, options.collision_resolution, checker);
        if ~isnan(otherIdx)
            found = true;
            connectA = otherIdx;
            connectB = newIdx;
            break;
        end
    end
end

if found
    pathStart = backtrack_tree(treeA, connectA);
    pathGoal = backtrack_tree(treeB, connectB);
    path = deduplicate_path([pathStart; flipud(pathGoal)]);
else
    path = [];
    iter = maxIter;
end

info = make_info(found, iter, treeA.nodes, treeA.parents, treeB.nodes, treeB.parents);
info.connect_start_index = connectA;
info.connect_goal_index = connectB;
end

function options = fill_rrt_connect_defaults(options, stepSize)
if ~isfield(options, "goal_bias") || isempty(options.goal_bias)
    options.goal_bias = 0.15;
end
if ~isfield(options, "collision_resolution") || isempty(options.collision_resolution)
    options.collision_resolution = max(stepSize / 2, 1e-3);
end
if ~isfield(options, "collision_checker") || isempty(options.collision_checker)
    options.collision_checker = @(q)is_collision_free(q);
end
if options.goal_bias < 0 || options.goal_bias > 1
    error("options.goal_bias must be in [0, 1].");
end
end

function limits = validate_joint_limits(limits, numJoints)
limits = double(limits);
if ~isequal(size(limits), [numJoints, 2])
    error("jointLimits must have size numJoints x 2.");
end
if any(limits(:, 1) >= limits(:, 2))
    error("Each joint limit must satisfy lower < upper.");
end
end

function ok = within_joint_limits(q, limits)
q = q(:).';
ok = all(q >= limits(:, 1).' - 1e-10) && all(q <= limits(:, 2).' + 1e-10);
end

function q = sample_joint_limits(limits)
q = limits(:, 1).' + rand(1, size(limits, 1)) .* (limits(:, 2).' - limits(:, 1).');
end

function [tree, newIdx] = extend_tree(tree, sample, stepSize, limits, resolution, checker)
[~, nearIdx] = min(vecnorm(tree.nodes - sample, 2, 2));
near = tree.nodes(nearIdx, :);
candidate = steer(near, sample, stepSize);
candidate = min(max(candidate, limits(:, 1).'), limits(:, 2).');

if norm(candidate - near) < 1e-12 || ~checker(candidate) || ...
        ~edge_collision_free(near, candidate, resolution, checker)
    newIdx = NaN;
    return;
end

tree.nodes = [tree.nodes; candidate]; %#ok<AGROW>
tree.parents = [tree.parents; nearIdx]; %#ok<AGROW>
newIdx = size(tree.nodes, 1);
end

function [tree, connectIdx] = connect_tree(tree, target, stepSize, limits, resolution, checker)
connectIdx = NaN;
while true
    [~, nearIdx] = min(vecnorm(tree.nodes - target, 2, 2));
    near = tree.nodes(nearIdx, :);
    candidate = steer(near, target, stepSize);
    candidate = min(max(candidate, limits(:, 1).'), limits(:, 2).');

    if norm(candidate - near) < 1e-12
        if norm(near - target) <= 1e-9
            connectIdx = nearIdx;
        end
        return;
    end
    if ~checker(candidate) || ~edge_collision_free(near, candidate, resolution, checker)
        return;
    end

    tree.nodes = [tree.nodes; candidate]; %#ok<AGROW>
    tree.parents = [tree.parents; nearIdx]; %#ok<AGROW>
    newIdx = size(tree.nodes, 1);

    if norm(candidate - target) <= 1e-9
        connectIdx = newIdx;
        return;
    end
end
end

function qNew = steer(qFrom, qTo, stepSize)
delta = qTo - qFrom;
distance = norm(delta);
if distance <= stepSize
    qNew = qTo;
else
    qNew = qFrom + stepSize * delta / distance;
end
end

function ok = edge_collision_free(q1, q2, resolution, checker)
distance = norm(q2 - q1);
numSamples = max(2, ceil(distance / resolution) + 1);
for i = 1:numSamples
    alpha = (i - 1) / (numSamples - 1);
    q = q1 + alpha * (q2 - q1);
    if ~checker(q)
        ok = false;
        return;
    end
end
ok = true;
end

function path = backtrack_tree(tree, idx)
path = tree.nodes(idx, :);
while tree.parents(idx) ~= 0
    idx = tree.parents(idx);
    path = [tree.nodes(idx, :); path]; %#ok<AGROW>
end
end

function path = deduplicate_path(path)
if isempty(path)
    return;
end
keep = true(size(path, 1), 1);
for i = 2:size(path, 1)
    keep(i) = norm(path(i, :) - path(i - 1, :)) > 1e-10;
end
path = path(keep, :);
end

function info = make_info(found, iterations, nodesA, parentsA, nodesB, parentsB)
info.found = found;
info.iterations = iterations;
info.start_tree_nodes = nodesA;
info.start_tree_parents = parentsA;
info.goal_tree_nodes = nodesB;
info.goal_tree_parents = parentsB;
info.nodes = [nodesA; nodesB];
info.parents = [parentsA; parentsB];
end
