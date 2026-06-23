function [trajectory, info] = smooth_trajectory(path, maxSmoothIterations, options)
%SMOOTH_TRAJECTORY Shortcut and interpolate a joint-space path.
%
% trajectory = smooth_trajectory(path)
% [trajectory, info] = smooth_trajectory(path, maxSmoothIterations, options)
%
% options:
%   collision_checker    函数句柄 @(q)isSafe
%   collision_resolution shortcut 边检查分辨率，默认 0.05
%   interpolation_step   输出轨迹相邻点最大距离，默认 0.04
%   method               "pchip" 或 "linear"，默认 "pchip"

if nargin < 2 || isempty(maxSmoothIterations)
    maxSmoothIterations = 50;
end
if nargin < 3
    options = struct();
end
options = fill_smooth_defaults(options);

path = double(path);
if isempty(path)
    trajectory = path;
    info.shortcut_path = path;
    info.method = options.method;
    return;
end
if size(path, 1) <= 1
    trajectory = path;
    info.shortcut_path = path;
    info.method = options.method;
    return;
end
if maxSmoothIterations < 0
    error("maxSmoothIterations must be non-negative.");
end

shortcutPath = shortcut_joint_path(path, maxSmoothIterations, options);
sampleStep = min(options.interpolation_step, options.collision_resolution);

if options.method == "pchip"
    candidate = interpolate_pchip(shortcutPath, sampleStep);
    if path_collision_free(candidate, options.collision_checker)
        trajectory = candidate;
        method = "pchip";
    else
        trajectory = interpolate_linear(shortcutPath, sampleStep);
        method = "linear_fallback";
    end
else
    trajectory = interpolate_linear(shortcutPath, sampleStep);
    method = "linear";
end

info.shortcut_path = shortcutPath;
info.method = method;
info.num_input_points = size(path, 1);
info.num_shortcut_points = size(shortcutPath, 1);
info.num_output_points = size(trajectory, 1);
end

function options = fill_smooth_defaults(options)
if ~isfield(options, "collision_checker") || isempty(options.collision_checker)
    options.collision_checker = @(q)is_collision_free(q);
end
if ~isfield(options, "collision_resolution") || isempty(options.collision_resolution)
    options.collision_resolution = 0.05;
end
if ~isfield(options, "interpolation_step") || isempty(options.interpolation_step)
    options.interpolation_step = 0.04;
end
if ~isfield(options, "method") || isempty(options.method)
    options.method = "pchip";
else
    options.method = string(options.method);
end
if options.collision_resolution <= 0 || options.interpolation_step <= 0
    error("collision_resolution and interpolation_step must be positive.");
end
end

function smooth = shortcut_joint_path(path, iterations, options)
smooth = path;
for k = 1:iterations
    if size(smooth, 1) <= 2
        break;
    end

    i = randi([1, size(smooth, 1) - 2]);
    j = randi([i + 2, size(smooth, 1)]);
    if edge_collision_free(smooth(i, :), smooth(j, :), options.collision_resolution, options.collision_checker)
        smooth = [smooth(1:i, :); smooth(j:end, :)];
    end
end
end

function trajectory = interpolate_pchip(path, step)
if size(path, 1) <= 2
    trajectory = interpolate_linear(path, step);
    return;
end

s = chord_parameter(path);
totalLength = s(end);
if totalLength < 1e-12
    trajectory = path(1, :);
    return;
end

numSamples = max(2, ceil(totalLength / step) + 1);
sQuery = linspace(0, totalLength, numSamples).';
trajectory = zeros(numSamples, size(path, 2));
for j = 1:size(path, 2)
    trajectory(:, j) = pchip(s, path(:, j), sQuery);
end
trajectory(1, :) = path(1, :);
trajectory(end, :) = path(end, :);
end

function trajectory = interpolate_linear(path, step)
trajectory = path(1, :);
for i = 1:(size(path, 1) - 1)
    q1 = path(i, :);
    q2 = path(i + 1, :);
    distance = norm(q2 - q1);
    numSteps = max(1, ceil(distance / step));
    for k = 1:numSteps
        alpha = k / numSteps;
        trajectory = [trajectory; q1 + alpha * (q2 - q1)]; %#ok<AGROW>
    end
end
end

function s = chord_parameter(path)
segmentLengths = vecnorm(diff(path, 1, 1), 2, 2);
s = [0; cumsum(segmentLengths)];
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

function ok = path_collision_free(path, checker)
ok = true;
for i = 1:size(path, 1)
    if ~checker(path(i, :))
        ok = false;
        return;
    end
end
end
