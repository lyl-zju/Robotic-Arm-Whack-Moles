function animate_robot(robot, traj, cfgRobot, cfgBoard, targetWorld, savePath)
if nargin < 4
    cfgBoard = [];
end
if nargin < 5
    targetWorld = [];
end
if nargin < 6
    savePath = "";
end

if strlength(string(savePath)) > 0 && exist(savePath, "file")
    delete(savePath);
end

fig = figure("Name", "Robot Hit Animation", "Color", "w");
ax = axes(fig);

numSamples = size(traj.q_des, 1);
maxAnimationFrames = 30;
frameStep = max(1, ceil(numSamples / maxAnimationFrames));
frameSamples = unique([1:frameStep:numSamples, numSamples]);
frameIndex = 1;
eeTrail = zeros(numSamples, 3);

for i = frameSamples
    qNow = traj.q_des(i, :);
    eePose = getTransform(robot, qNow, cfgRobot.end_effector);
    eeTrail(i, :) = tform2trvec(eePose);

    show(robot, qNow, "Parent", ax, "Frames", "off", "Visuals", "on", "PreservePlot", false);
    hold(ax, "on");

    if ~isempty(cfgBoard)
        draw_world_board(ax, cfgBoard, targetWorld);
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
    title(ax, sprintf("UR5 hit simulation, t = %.2f s, ee = %s", traj.t(i), cfgRobot.end_effector));
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

function draw_world_board(ax, cfgBoard, targetWorld)
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
    targetPoint = [targetWorld(1), targetWorld(2), cfgBoard.z_hit];
    scatter3(ax, targetPoint(1), targetPoint(2), targetPoint(3), ...
        110, [0.9 0.05 0.05], "filled");
end
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
