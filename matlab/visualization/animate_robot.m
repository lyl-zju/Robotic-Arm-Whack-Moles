function animate_robot(robot, traj, cfgRobot, cfgBoard, targetWorld, savePath, impactResult)
if nargin < 4
    cfgBoard = [];
end
if nargin < 5
    targetWorld = [];
end
if nargin < 6
    savePath = "";
end
if nargin < 7
    impactResult = [];
end

if strlength(string(savePath)) > 0 && exist(savePath, "file")
    delete(savePath);
end

fig = figure("Name", "Robot Hit Animation", "Color", "w");
ax = axes(fig);
forceHud = [];

numSamples = size(traj.q_des, 1);
if isempty(impactResult)
    maxAnimationFrames = 20;
else
    maxAnimationFrames = 28;
    forceHud = create_force_hud(fig);
end
frameStep = max(1, ceil(numSamples / maxAnimationFrames));
frameSamples = unique([1:frameStep:numSamples, numSamples]);
frameIndex = 1;
eeTrail = zeros(numSamples, 3);

for i = frameSamples
    qNow = traj.q_des(i, :);
    eePose = getTransform(robot, qNow, cfgRobot.end_effector);
    eeTrail(i, :) = tform2trvec(eePose);
    moleState = mole_state_from_result(impactResult, i);

    show(robot, qNow, "Parent", ax, "Frames", "off", "Visuals", "on", "PreservePlot", false);
    hold(ax, "on");

    if ~isempty(cfgBoard)
        draw_world_board(ax, cfgBoard, targetWorld, moleState);
    end

    validTrail = any(eeTrail ~= 0, 2);
    plot3(ax, eeTrail(validTrail, 1), eeTrail(validTrail, 2), eeTrail(validTrail, 3), ...
        "LineWidth", 1.5, "Color", [0.05 0.35 0.9]);

    axis(ax, "equal");
    grid(ax, "on");
    [xLimits, yLimits] = animation_xy_limits(cfgBoard);
    xlim(ax, xLimits);
    ylim(ax, yLimits);
    zlim(ax, [-0.05, 0.7]);
    view(ax, 135, 25);
    xlabel(ax, "x / m");
    ylabel(ax, "y / m");
    zlabel(ax, "z / m");
    update_force_hud(forceHud, moleState);
    title(ax, frame_title(traj.t(i), cfgRobot, moleState));
    drawnow;

    if strlength(string(savePath)) > 0
        write_gif_frame(fig, savePath, frameIndex, 0.04);
        frameIndex = frameIndex + 1;
    end
end
end

function [xLimits, yLimits] = animation_xy_limits(cfgBoard)
if isempty(cfgBoard)
    xLimits = [0.0, 0.8];
    yLimits = [-0.4, 0.4];
    return;
end

margin = 0.18;
xLimits = cfgBoard.center_world(1) + [-cfgBoard.width / 2 - margin, cfgBoard.width / 2 + margin];
yLimits = cfgBoard.center_world(2) + [-cfgBoard.height / 2 - margin, cfgBoard.height / 2 + margin];
xLimits(1) = min(0.0, xLimits(1));
end

function draw_world_board(ax, cfgBoard, targetWorld, moleState)
if nargin < 4
    moleState = [];
end

halfWidth = cfgBoard.width / 2;
halfHeight = cfgBoard.height / 2;
boardCorners = [
    -halfWidth, -halfHeight, 0;
     halfWidth, -halfHeight, 0;
     halfWidth,  halfHeight, 0;
    -halfWidth,  halfHeight, 0
];
worldCorners = (cfgBoard.R_world_board * boardCorners.').' + cfgBoard.center_world;

patch(ax, ...
    "XData", worldCorners(:, 1), ...
    "YData", worldCorners(:, 2), ...
    "ZData", worldCorners(:, 3), ...
    "FaceColor", [0.92 0.92 0.92], ...
    "FaceAlpha", 0.45, ...
    "EdgeColor", [0.2 0.2 0.2], ...
    "LineWidth", 1.2);

holePoints = cfgBoard.holes_world;
holePoints(:, 3) = cfgBoard.z_hit;
scatter3(ax, holePoints(:, 1), holePoints(:, 2), holePoints(:, 3), ...
    42, [0.15 0.15 0.15], "filled");

for k = 1:size(holePoints, 1)
    text(ax, holePoints(k, 1), holePoints(k, 2), holePoints(k, 3) + 0.025, string(k), ...
        "HorizontalAlignment", "center", "FontSize", 8);
end

if ~isempty(targetWorld)
    draw_mole(ax, cfgBoard, targetWorld, moleState);
end
end

function state = mole_state_from_result(result, sampleIndex)
state.has_force = false;
state.t = NaN;
state.drop = 0.0;
state.force_ratio = 0.0;
state.force = 0.0;
state.filtered_force = 0.0;
state.peak_force = 0.0;
state.threshold = 0.0;
state.target_force = 0.0;
state.phase = "";
state.success = false;
if isempty(result) || ~isfield(result, "force")
    return;
end

force = result.force;
sampleIndex = min(sampleIndex, numel(force.t));
state.has_force = true;
state.t = force.t(sampleIndex);

if isfield(force, "penetration") && numel(force.penetration) >= sampleIndex
    state.drop = max(0.0, force.penetration(sampleIndex));
end
if isfield(force, "normal") && isfield(force, "threshold") && force.threshold > 0 && ...
        numel(force.normal) >= sampleIndex
    state.force = max(0.0, force.normal(sampleIndex));
    state.threshold = force.threshold;
    state.force_ratio = min(1.0, max(0.0, state.force / state.threshold));
    state.peak_force = max(force.normal(1:sampleIndex));
end
if isfield(force, "filtered") && numel(force.filtered) >= sampleIndex
    state.filtered_force = max(0.0, force.filtered(sampleIndex));
else
    state.filtered_force = state.force;
end
if isfield(force, "desired") && numel(force.desired) >= sampleIndex
    state.target_force = force.desired(sampleIndex);
elseif isfield(force, "target")
    state.target_force = force.target;
end
if isfield(result, "control") && isfield(result.control, "phase") && ...
        numel(result.control.phase) >= sampleIndex
    state.phase = string(result.control.phase(sampleIndex));
end
if isfield(force, "success_time") && isfinite(force.success_time)
    tNow = force.t(sampleIndex);
    if tNow >= force.success_time
        u = min(1.0, (tNow - force.success_time) / 0.18);
        state.drop = max(state.drop, 0.032 * smoothstep(u));
        state.success = true;
    end
end
end

function hud = create_force_hud(fig)
figure(fig);
hud.text = annotation(fig, "textbox", [0.02 0.68 0.32 0.28], ...
    "String", "", ...
    "FitBoxToText", "off", ...
    "FontName", "Consolas", ...
    "FontSize", 9, ...
    "FontWeight", "bold", ...
    "Color", [0.04 0.05 0.06], ...
    "BackgroundColor", [1.0 1.0 1.0], ...
    "EdgeColor", [0.28 0.28 0.28], ...
    "LineWidth", 0.8, ...
    "Margin", 6);
hud.barBack = annotation(fig, "rectangle", [0.04 0.645 0.28 0.024], ...
    "FaceColor", [0.86 0.86 0.86], ...
    "Color", [0.30 0.30 0.30], ...
    "LineWidth", 0.7);
hud.barFill = annotation(fig, "rectangle", [0.04 0.645 0.001 0.024], ...
    "FaceColor", [0.85 0.18 0.12], ...
    "Color", "none");
end

function update_force_hud(hud, moleState)
if isempty(hud) || ~moleState.has_force
    return;
end

phaseText = char(moleState.phase);
if isempty(phaseText)
    phaseText = 'n/a';
end

threshold = max(moleState.threshold, eps);
fillRatio = min(1.0, max(0.0, moleState.force / threshold));
barColor = [0.85 0.18 0.12];
if moleState.force >= moleState.threshold
    barColor = [0.12 0.62 0.22];
end

hudText = sprintf([ ...
    'contact force  %5.2f N\n', ...
    'filtered       %5.2f N\n', ...
    'peak           %5.2f N\n', ...
    'threshold      %5.2f N\n', ...
    'target         %5.2f N\n', ...
    'phase          %s'], ...
    moleState.force, moleState.filtered_force, moleState.peak_force, ...
    moleState.threshold, moleState.target_force, phaseText);

set(hud.text, "String", hudText);
set(hud.barFill, ...
    "Position", [0.04 0.645 0.28 * fillRatio 0.024], ...
    "FaceColor", barColor);
end

function titleText = frame_title(t, cfgRobot, moleState)
if moleState.has_force
    titleText = sprintf('UR5 impact force control, t = %.2f s, F_n = %.2f N, peak = %.2f N', ...
        t, moleState.force, moleState.peak_force);
else
    titleText = sprintf('UR5 hit simulation, t = %.2f s, ee = %s', t, cfgRobot.end_effector);
end
end

function draw_mole(ax, cfgBoard, targetWorld, moleState)
if isempty(moleState)
    moleState.drop = 0.0;
    moleState.force_ratio = 0.0;
    moleState.success = false;
end

radius = max(0.014, 0.85 * cfgBoard.hit_threshold);
holeRadius = radius * 1.35;
holeZ = cfgBoard.center_world(3) + 0.0015;
theta = linspace(0, 2 * pi, 44);

fill3(ax, ...
    targetWorld(1) + holeRadius * cos(theta), ...
    targetWorld(2) + holeRadius * sin(theta), ...
    holeZ * ones(size(theta)), ...
    [0.04 0.04 0.045], ...
    "EdgeColor", [0.01 0.01 0.01], ...
    "FaceAlpha", 0.88);

maxVisualDrop = 0.034;
drop = min(maxVisualDrop, max(0.0, moleState.drop));
topZ = cfgBoard.z_hit - drop;
bottomZ = cfgBoard.center_world(3) - 0.024 - 0.25 * drop;
if topZ <= bottomZ + 0.002
    topZ = bottomZ + 0.002;
end

[cx, cy, cz] = cylinder(radius, 36);
cz = bottomZ + (topZ - bottomZ) * cz;
surf(ax, ...
    targetWorld(1) + cx, ...
    targetWorld(2) + cy, ...
    cz, ...
    "FaceColor", [0.82 0.10 0.07], ...
    "EdgeColor", "none", ...
    "FaceAlpha", 0.98);

capColor = [0.98, 0.22 + 0.35 * moleState.force_ratio, 0.10];
fill3(ax, ...
    targetWorld(1) + radius * cos(theta), ...
    targetWorld(2) + radius * sin(theta), ...
    topZ * ones(size(theta)), ...
    capColor, ...
    "EdgeColor", [0.48 0.05 0.04], ...
    "LineWidth", 0.6);

if moleState.success
    scatter3(ax, targetWorld(1), targetWorld(2), cfgBoard.z_hit + 0.03, ...
        38, [0.10 0.65 0.18], "filled");
end
end

function y = smoothstep(x)
x = min(max(x, 0.0), 1.0);
y = x * x * (3 - 2 * x);
end

function write_gif_frame(fig, savePath, frameIndex, frameDelay)
frame = getframe(fig);
[indexedImage, colorMap] = rgb2ind(frame2im(frame), 256);

if frameIndex == 1
    imwrite(indexedImage, colorMap, savePath, "gif", "LoopCount", inf, "DelayTime", frameDelay);
else
    imwrite(indexedImage, colorMap, savePath, "gif", "WriteMode", "append", "DelayTime", frameDelay);
end
end
