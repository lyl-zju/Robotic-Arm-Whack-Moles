clear; clc; close all;

% Visual test for the new joint-space RRT-Connect planner.
% Run from MATLAB:
%   run("matlab/test/test_joint_rrt_connect_visual.m")

rootDir = fileparts(fileparts(fileparts(mfilename("fullpath"))));
addpath(genpath(fullfile(rootDir, "matlab")));

fprintf("=== Joint-space RRT-Connect visual test ===\n");

run_interactive_ur5_strike_visual_test(rootDir);

fprintf("Visual test finished.\n");

function run_interactive_ur5_strike_visual_test(rootDir)
cfgRobot = config_robot();
cfgBoard = config_board();
cfgController = config_controller();
cfgController.path_type = "joint_rrt_connect";
cfgController.motion_mode = normalize_motion_mode("continuous");
cfgController.dt = 0.01;
cfgController.segment_time = 0.015;

robot = build_robot(cfgRobot);
obstacles3d = make_demo_obstacles_3d(cfgBoard);

currentJoints = cfgRobot.home_configuration;
lastTraj = [];

options.max_iter = 2200;
options.step_size = 0.16;
options.goal_bias = 0.18;
options.collision_resolution = 0.05;
options.max_smooth_iterations = 80;
options.interpolation_step = 0.12;
options.interpolation_method = "pchip";
options.seed = 5;
options.collision_options.min_z = -0.05;
options.collision_options.safety_margin = 0.015;
options.collision_options.link_radius = 0.035;
options.collision_options.link_sample_resolution = 0.020;
options.collision_options.obstacles = obstacles3d;

show_robot_preview(robot, currentJoints, cfgRobot, cfgBoard, obstacles3d);
show_joint_profiles_preview(currentJoints);

fig = figure("Name", "Click target for joint RRT-Connect test", "Color", "w", "NumberTitle", "off");
ax = axes(fig);
draw_board(cfgBoard, []);
title(ax, "Click a target on the board. 3-D obstacles are shown in the robot window.");
hold(ax, "on");
targetMarker = plot(ax, NaN, NaN, "ro", "MarkerSize", 16, "LineWidth", 2.5);
set(fig, "KeyPressFcn", @(src, event)setappdata(src, "stop", strcmp(event.Key, "escape")));
set(fig, "WindowButtonDownFcn", @(src, ~)on_board_click(src, ax, cfgBoard));
setappdata(fig, "stop", false);
setappdata(fig, "has_target", false);
setappdata(fig, "target_board", []);
setappdata(fig, "is_planning", false);

fprintf("Click a target on the board window. 3-D obstacle setup is in make_demo_obstacles_3d().\n");
figure(fig);

while isgraphics(fig) && ~logical(getappdata(fig, "stop"))
    drawnow;
    if ~logical(getappdata(fig, "has_target"))
        pause(0.03);
        continue;
    end
    clickedPoint = getappdata(fig, "target_board");
    setappdata(fig, "has_target", false);
    setappdata(fig, "is_planning", true);

    targetWorld = board_to_world(clickedPoint, cfgBoard);
    set(targetMarker, "XData", clickedPoint(1), "YData", clickedPoint(2));
    title(ax, "Planning joint-space RRT-Connect trajectory...", "Color", [0.0 0.35 0.8]);
    drawnow;

    fprintf("Planning to board target [%.3f %.3f]...\n", clickedPoint(1), clickedPoint(2));
    tic;
    try
        [traj, planInfo] = plan_strike_trajectory( ...
            robot, currentJoints, targetWorld, cfgRobot, cfgBoard, cfgController, options);
    catch err
        warning("test_joint_rrt_connect_visual:PlanningFailed", "%s", err.message);
        title(ax, "Planning failed. Choose another target or adjust obstacles/options.", ...
            "Color", [0.85 0.05 0.05]);
        setappdata(fig, "is_planning", false);
        drawnow;
        continue;
    end
    planningTime = toc;

    fprintf("UR5 plan finished in %.3f s. raw=%d, smooth=%d, q_des=%d\n", ...
        planningTime, size(planInfo.raw_path, 1), size(planInfo.smooth_path, 1), size(traj.q_des, 1));

    rawEePath = joint_path_to_ee_path(robot, planInfo.raw_path, cfgRobot);
    smoothEePath = joint_path_to_ee_path(robot, planInfo.smooth_path, cfgRobot);
    trajEePath = joint_path_to_ee_path(robot, traj.q_des, cfgRobot);

    animate_joint_rrt_result(robot, traj, cfgBoard, targetWorld, obstacles3d, fig, ...
        rawEePath, smoothEePath, trajEePath);

    plot_joint_profiles(planInfo, traj);

    currentJoints = traj.q_des(end, :);
    lastTraj = traj;

    title(ax, "Trajectory executed. Click another safe target, or press Esc to finish.", ...
        "Color", [0.0 0.45 0.15]);
    setappdata(fig, "is_planning", false);
    drawnow;
end

if ~isempty(lastTraj)
    % Export a separate CSV so the test result can also be replayed by PyBullet
    % without overwriting the normal main_sim output.
    outPath = fullfile(rootDir, "shared", "q_traj_joint_rrt_test.csv");
    export_q_traj(lastTraj, outPath);
    fprintf("Exported last test trajectory: %s\n", outPath);
end
end

function show_robot_preview(robot, q, ~, cfgBoard, obstacles3d)
fig = figure("Name", "UR5 joint-space RRT-Connect strike test", "Color", "w");
ax = axes(fig);
show(robot, q, "Parent", ax, "Frames", "off", "Visuals", "on", "PreservePlot", false);
hold(ax, "on");
draw_world_board_for_test(ax, cfgBoard, [], obstacles3d);
axis(ax, "equal");
grid(ax, "on");
xlim(ax, [0.0, cfgBoard.center_world(1) + cfgBoard.width / 2 + 0.20]);
ylim(ax, cfgBoard.center_world(2) + [-cfgBoard.height / 2 - 0.22, cfgBoard.height / 2 + 0.22]);
zlim(ax, [-0.05, 0.70]);
view(ax, 135, 25);
xlabel(ax, "x / m");
ylabel(ax, "y / m");
zlabel(ax, "z / m");
title(ax, "UR5 initial pose with 3-D obstacles");
drawnow;
end

function show_joint_profiles_preview(currentJoints)
fig = findobj("Type", "figure", "Name", "Joint-space path profiles");
if isempty(fig) || ~isgraphics(fig(1))
    fig = figure("Name", "Joint-space path profiles", "Color", "w");
else
    fig = fig(1);
    clf(fig);
end

layout = tiledlayout(fig, 2, 1);

axPath = nexttile(layout);
plot(currentJoints, "o-", "LineWidth", 1.4);
grid on;
xlabel("joint index");
ylabel("joint angle / rad");
title("Initial joint configuration");

axTraj = nexttile(layout);
plot(0, currentJoints, "o", "LineWidth", 1.4);
grid on;
xlabel("time / s");
ylabel("joint angle / rad");
title("Executable trajectory will appear after the first target");
setappdata(fig, "ax_path", axPath);
setappdata(fig, "ax_traj", axTraj);
end

function animate_joint_rrt_result(robot, traj, cfgBoard, targetWorld, obstacles3d, boardFig, rawEePath, smoothEePath, trajEePath)
fig = findobj("Type", "figure", "Name", "UR5 joint-space RRT-Connect strike test");
if isempty(fig) || ~isgraphics(fig(1))
    fig = figure("Name", "UR5 joint-space RRT-Connect strike test", "Color", "w");
    ax = axes(fig);
else
    fig = fig(1);
    figure(fig);
    ax = findobj(fig, "Type", "axes");
    if isempty(ax)
        ax = axes(fig);
    else
        ax = ax(1);
    end
end

numSamples = size(traj.q_des, 1);
maxFrames = 45;
frameStep = max(1, ceil(numSamples / maxFrames));
frameIndices = unique([1:frameStep:numSamples, numSamples]);

for frameIdx = frameIndices
    if ~isgraphics(fig)
        break;
    end

    qNow = traj.q_des(frameIdx, :);
    show(robot, qNow, "Parent", ax, "Frames", "off", "Visuals", "on", "PreservePlot", false);
    hold(ax, "on");

    draw_world_board_for_test(ax, cfgBoard, targetWorld, obstacles3d);
    plot3(ax, rawEePath(:, 1), rawEePath(:, 2), rawEePath(:, 3), ...
        "Color", [0.55 0.55 0.55], "LineStyle", "--", "LineWidth", 1.2);
    plot3(ax, smoothEePath(:, 1), smoothEePath(:, 2), smoothEePath(:, 3), ...
        "Color", [0.05 0.25 0.90], "LineWidth", 2.0);
    plot3(ax, trajEePath(1:frameIdx, 1), trajEePath(1:frameIdx, 2), trajEePath(1:frameIdx, 3), ...
        "Color", [0.0 0.55 0.20], "LineWidth", 1.8);

    axis(ax, "equal"); grid(ax, "on");
    xlim(ax, [0.0, cfgBoard.center_world(1) + cfgBoard.width / 2 + 0.20]);
    ylim(ax, cfgBoard.center_world(2) + [-cfgBoard.height / 2 - 0.22, cfgBoard.height / 2 + 0.22]);
    zlim(ax, [-0.05, 0.70]);
    view(ax, 135, 25);
    xlabel(ax, "x / m"); ylabel(ax, "y / m"); zlabel(ax, "z / m");
    title(ax, sprintf("Joint RRT-Connect strike, t = %.2f s", traj.t(frameIdx)));
    legend(ax, ["raw RRT EE path", "smoothed EE path", "executed EE trail"], ...
        "Location", "northeastoutside");
    drawnow;
    pause(0.004);
end
if isgraphics(boardFig)
    figure(boardFig);
end
end

function plot_joint_profiles(planInfo, traj)
fig = findobj("Type", "figure", "Name", "Joint-space path profiles");
if isempty(fig) || ~isgraphics(fig(1))
    return;
end
fig = fig(1);

if isappdata(fig, "ax_path") && isgraphics(getappdata(fig, "ax_path")) && ...
        isappdata(fig, "ax_traj") && isgraphics(getappdata(fig, "ax_traj"))
    axPath = getappdata(fig, "ax_path");
    axTraj = getappdata(fig, "ax_traj");
else
    return;
end

cla(axPath);
plot(axPath, planInfo.raw_path, "--", "LineWidth", 1.0);
hold(axPath, "on");
plot(axPath, planInfo.smooth_path, "LineWidth", 1.6);
grid(axPath, "on");
xlabel(axPath, "path sample index"); ylabel(axPath, "joint angle / rad");
title(axPath, "Raw and smoothed joint-space paths");

cla(axTraj);
plot(axTraj, traj.t, traj.q_des, "LineWidth", 1.3);
grid(axTraj, "on");
xlabel(axTraj, "time / s"); ylabel(axTraj, "joint angle / rad");
title(axTraj, "Executable quintic joint trajectory");
drawnow limitrate nocallbacks;
end

function eePath = joint_path_to_ee_path(robot, jointPath, cfgRobot)
eePath = zeros(size(jointPath, 1), 3);
for i = 1:size(jointPath, 1)
    tform = getTransform(robot, jointPath(i, :), cfgRobot.end_effector);
    eePath(i, :) = tform2trvec(tform);
end
end

function obstacles = make_demo_obstacles_3d(cfgBoard)
% Edit this function to set real 3-D world obstacles.
% The offsets below are in the board frame: [board_x, board_y, height_above_board].
% board_x/board_y use the same coordinates as the click window, and height is
% measured upward from cfgBoard.center_world(3).
% sphere:   center + radius
% box:      center + size [x y z]
% cylinder: center + radius + height, vertical by default
obstacles = [
    struct("type", "sphere", ...
        "center", board_offset_to_world(cfgBoard, [-0.15, 0, 0.20]), ...
        "radius", 0.040, "size", [], "height", [], "axis", [], "rotation", [])
    struct("type", "box", ...
        "center", board_offset_to_world(cfgBoard, [0.02, 0.2, 0.18]), ...
        "size", [0.075, 0.070, 0.120], "radius", [], "height", [], "axis", [], "rotation", [])
    struct("type", "cylinder", ...
        "center", board_offset_to_world(cfgBoard, [0.20, 0, 0.12]), ...
        "radius", 0.032, "height", 0.150, "size", [], "axis", [0, 0, 1], "rotation", [])
];
end

function pointWorld = board_offset_to_world(cfgBoard, offsetBoard3)
pointWorld = (cfgBoard.R_world_board * offsetBoard3(:)).' + cfgBoard.center_world;
end

function on_board_click(fig, ax, cfgBoard)
if ~isgraphics(fig) || ~strcmp(get(fig, "SelectionType"), "normal")
    return;
end
if logical(getappdata(fig, "stop"))
    return;
end
if isappdata(fig, "is_planning") && logical(getappdata(fig, "is_planning"))
    return;
end

currentPoint = get(ax, "CurrentPoint");
candidate = currentPoint(1, 1:2);
if is_inside_board(candidate, cfgBoard)
    setappdata(fig, "target_board", candidate);
    setappdata(fig, "has_target", true);
end
end

function inside = is_inside_board(pointBoard, cfgBoard)
inside = pointBoard(1) >= -cfgBoard.width / 2 && pointBoard(1) <= cfgBoard.width / 2 && ...
    pointBoard(2) >= -cfgBoard.height / 2 && pointBoard(2) <= cfgBoard.height / 2;
end

function draw_world_board_for_test(ax, cfgBoard, targetWorld, obstacles3d)
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
    38, [0.15 0.15 0.15], "filled");

if ~isempty(targetWorld)
    targetPoint = [targetWorld(1), targetWorld(2), cfgBoard.z_hit];
    scatter3(ax, targetPoint(1), targetPoint(2), targetPoint(3), ...
        130, [0.9 0.05 0.05], "filled");
end

draw_world_obstacles_for_test(ax, obstacles3d);
end

function draw_world_obstacles_for_test(ax, obstacles)
for k = 1:numel(obstacles)
    obstacle = obstacles(k);
    switch string(obstacle.type)
        case "sphere"
            [x, y, z] = sphere(24);
            surf(ax, ...
                obstacle.center(1) + obstacle.radius * x, ...
                obstacle.center(2) + obstacle.radius * y, ...
                obstacle.center(3) + obstacle.radius * z, ...
                "FaceColor", [0.85 0.18 0.10], ...
                "FaceAlpha", 0.35, ...
                "EdgeColor", "none");
        case "box"
            draw_box_obstacle(ax, obstacle.center, obstacle.size);
        case "cylinder"
            [x, y, z] = cylinder(obstacle.radius, 32);
            z = (z - 0.5) * obstacle.height;
            surf(ax, ...
                obstacle.center(1) + x, ...
                obstacle.center(2) + y, ...
                obstacle.center(3) + z, ...
                "FaceColor", [0.85 0.18 0.10], ...
                "FaceAlpha", 0.35, ...
                "EdgeColor", "none");
        otherwise
            error("Unknown obstacle type: %s", obstacle.type);
    end
end
end

function draw_box_obstacle(ax, center, sizeVec)
half = sizeVec / 2;
vertices = [
    -half(1), -half(2), -half(3);
     half(1), -half(2), -half(3);
     half(1),  half(2), -half(3);
    -half(1),  half(2), -half(3);
    -half(1), -half(2),  half(3);
     half(1), -half(2),  half(3);
     half(1),  half(2),  half(3);
    -half(1),  half(2),  half(3)
] + center;
faces = [
    1 2 3 4;
    5 6 7 8;
    1 2 6 5;
    2 3 7 6;
    3 4 8 7;
    4 1 5 8
];
patch(ax, ...
    "Vertices", vertices, ...
    "Faces", faces, ...
    "FaceColor", [0.85 0.18 0.10], ...
    "FaceAlpha", 0.30, ...
    "EdgeColor", [0.75 0.10 0.06], ...
    "LineWidth", 0.8);
end
