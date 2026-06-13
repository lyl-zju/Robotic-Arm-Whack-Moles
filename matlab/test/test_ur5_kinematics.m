clear; clc; close all;

rootDir = fileparts(fileparts(mfilename("fullpath")));
addpath(genpath(fullfile(rootDir, "matlab")));

fprintf("=== UR5 forward/inverse kinematics test ===\n");
fprintf("Project root: %s\n", rootDir);

cfgRobot = config_robot();
cfgBoard = config_board();
robot = build_robot(cfgRobot);

positionTol = 2e-3;
orientationTol = deg2rad(2);

homeQ = cfgRobot.home_configuration;
expectedDofs = numel(homeConfiguration(robot));
if numel(homeQ) ~= expectedDofs
    error("home_configuration has %d values, but robot expects %d DOF.", numel(homeQ), expectedDofs);
end

homePose = getTransform(robot, homeQ, cfgRobot.end_effector);
homePosition = tform2trvec(homePose);
homeEulerZyx = tform2eul(homePose, "ZYX");

fprintf("\nForward kinematics at home_configuration:\n");
fprintf("  q_home = [%s] rad\n", join(string(round(homeQ, 4)), ", "));
fprintf("  ee position = [%.4f %.4f %.4f] m\n", homePosition(1), homePosition(2), homePosition(3));
fprintf("  ee euler ZYX = [%.4f %.4f %.4f] rad\n", homeEulerZyx(1), homeEulerZyx(2), homeEulerZyx(3));

targetPoints = zeros(size(cfgBoard.holes_world));
targetNames = strings(size(cfgBoard.holes_world, 1), 1);
for i = 1:size(cfgBoard.holes_world, 1)
    xy = cfgBoard.holes_world(i, 1:2);
    targetPoints(i, :) = [xy, cfgBoard.z_hit];
    targetNames(i) = sprintf("hole_%02d", i);
end

targetRotation = tform2rotm(eul2tform(cfgRobot.fixed_tool_eul_zyx, "ZYX"));
positionErrors = zeros(size(targetPoints, 1), 1);
orientationErrors = zeros(size(targetPoints, 1), 1);
ikSolutions = zeros(size(targetPoints, 1), expectedDofs);

fprintf("\nInverse kinematics residuals checked by forward kinematics:\n");
fprintf("  %-14s %-25s %-14s %-14s\n", "target", "point [m]", "pos_err [m]", "rot_err [deg]");

qSeed = homeQ;
for i = 1:size(targetPoints, 1)
    point = targetPoints(i, :);
    q = solve_ik(robot, point, cfgRobot, qSeed);
    pose = getTransform(robot, q, cfgRobot.end_effector);

    actualPosition = tform2trvec(pose);
    actualRotation = tform2rotm(pose);

    positionErrors(i) = norm(actualPosition - point);
    orientationErrors(i) = rotation_angle_error(actualRotation, targetRotation);
    ikSolutions(i, :) = q;

    fprintf("  %-14s [%6.3f %6.3f %6.3f]   %-14.6g %-14.6g\n", ...
        targetNames(i), point(1), point(2), point(3), ...
        positionErrors(i), rad2deg(orientationErrors(i)));

    qSeed = q;
end

maxPositionError = max(positionErrors);
maxOrientationError = max(orientationErrors);

fprintf("\nSummary:\n");
fprintf("  max position error = %.6g m\n", maxPositionError);
fprintf("  max orientation error = %.6g deg\n", rad2deg(maxOrientationError));

if any(~isfinite(ikSolutions), "all")
    error("IK returned NaN or Inf in at least one solution.");
end

if maxPositionError > positionTol
    error("IK position error %.6g m exceeds tolerance %.6g m.", maxPositionError, positionTol);
end

if maxOrientationError > orientationTol
    error("IK orientation error %.6g deg exceeds tolerance %.6g deg.", ...
        rad2deg(maxOrientationError), rad2deg(orientationTol));
end

figDir = fullfile(rootDir, "results", "figures");
if ~exist(figDir, "dir")
    mkdir(figDir);
end

gifPath = fullfile(figDir, "ur5_ik_grid_test.gif");
animate_ik_solutions(robot, homeQ, ikSolutions, targetPoints, targetNames, cfgRobot, gifPath);

fprintf("[PASS] UR5 forward/inverse kinematics residuals are within tolerance.\n");
fprintf("Saved IK test animation: %s\n", gifPath);

function angle = rotation_angle_error(actualRotation, targetRotation)
rotationDelta = targetRotation.' * actualRotation;
cosAngle = (trace(rotationDelta) - 1) / 2;
cosAngle = min(1, max(-1, cosAngle));
angle = acos(cosAngle);
end

function animate_ik_solutions(robot, homeQ, ikSolutions, targetPoints, targetNames, cfgRobot, gifPath)
framesPerMove = 20;
holdFrames = 6;
frameDelay = 0.04;

if exist(gifPath, "file")
    delete(gifPath);
end

fig = figure("Name", "UR5 IK grid animation", "Color", "w");
ax = axes(fig);
allQ = [homeQ; ikSolutions];
frameIndex = 1;

for targetIndex = 1:size(ikSolutions, 1)
    qStart = allQ(targetIndex, :);
    qStop = allQ(targetIndex + 1, :);

    for step = 1:(framesPerMove + holdFrames)
        if step <= framesPerMove
            ratio = (step - 1) / max(1, framesPerMove - 1);
            ratio = 10 * ratio^3 - 15 * ratio^4 + 6 * ratio^5;
            qNow = qStart + ratio * (qStop - qStart);
        else
            qNow = qStop;
        end

        show_animation_frame(robot, qNow, targetPoints, targetNames, targetIndex, cfgRobot, ax);
        write_gif_frame(fig, gifPath, frameIndex, frameDelay);
        frameIndex = frameIndex + 1;
    end
end
end

function show_animation_frame(robot, qNow, targetPoints, targetNames, targetIndex, cfgRobot, ax)
show(robot, qNow, "Parent", ax, "Frames", "off", "Visuals", "on", "PreservePlot", false);
hold(ax, "on");
scatter3(ax, targetPoints(:, 1), targetPoints(:, 2), targetPoints(:, 3), ...
    42, [0.25 0.25 0.25], "filled");
scatter3(ax, targetPoints(targetIndex, 1), targetPoints(targetIndex, 2), targetPoints(targetIndex, 3), ...
    90, [0.9 0.1 0.1], "filled");
text(ax, targetPoints(:, 1), targetPoints(:, 2), targetPoints(:, 3) + 0.025, targetNames, ...
    "FontSize", 8, "HorizontalAlignment", "center");
axis(ax, "equal");
grid(ax, "on");
xlim(ax, [0.15, 0.75]);
ylim(ax, [-0.35, 0.35]);
zlim(ax, [-0.05, 0.55]);
view(ax, 135, 25);
xlabel(ax, "x [m]");
ylabel(ax, "y [m]");
zlabel(ax, "z [m]");
title(ax, sprintf("UR5 IK pose: %s, ee = %s", targetNames(targetIndex), cfgRobot.end_effector));
drawnow;
end

function write_gif_frame(fig, gifPath, frameIndex, frameDelay)
frame = getframe(fig);
[indexedImage, colorMap] = rgb2ind(frame2im(frame), 256);

if frameIndex == 1
    imwrite(indexedImage, colorMap, gifPath, "gif", "LoopCount", inf, "DelayTime", frameDelay);
else
    imwrite(indexedImage, colorMap, gifPath, "gif", "WriteMode", "append", "DelayTime", frameDelay);
end
end
