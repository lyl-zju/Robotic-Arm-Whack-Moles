requestedHitMotionMode = "";
if exist("hitMotionMode", "var")
    requestedHitMotionMode = hitMotionMode;
elseif exist("motionMode", "var")
    requestedHitMotionMode = motionMode;
end
clearvars -except requestedHitMotionMode; clc; close all;

rootDir = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(rootDir, "matlab")));

cfgRobot = config_robot();
cfgBoard = config_board();
cfgController = config_controller();
cfgController = apply_motion_mode_override(cfgController, requestedHitMotionMode);
cfgForce = config_force();

robot = build_robot(cfgRobot);
target = generate_random_target(cfgBoard);

fprintf("Random target id: %d, world = [%.3f %.3f %.3f]\n", ...
    target.id, target.position_world(1), target.position_world(2), target.position_world(3));
fprintf("Hit motion mode: %s\n", cfgController.motion_mode);

result = execute_hit_target(robot, target.position_world, cfgRobot, cfgBoard, cfgController, cfgForce);
result.target_id = target.id;
result.target_mode = target.mode;

figDir = fullfile(rootDir, "results", "figures");
if ~exist(figDir, "dir")
    mkdir(figDir);
end

videoDir = fullfile(rootDir, "results", "videos");
if ~exist(videoDir, "dir")
    mkdir(videoDir);
end

figure("Name", "Random Target Board", "Color", "w");
draw_board(cfgBoard, target.position_board);
title(sprintf("Random target %d on whac-a-mole board", target.id));

animate_robot(robot, result.traj, cfgRobot, cfgBoard, target.position_world, ...
    fullfile(videoDir, "random_target_animation.gif"));

plot_results(result, fullfile(figDir, "random_target_tracking.png"));

save_result_json(result, fullfile(rootDir, "shared", "result.json"));
export_q_traj(result.traj, fullfile(rootDir, "shared", "q_traj.csv"));

fprintf("Peak contact force: %.2f N, threshold: %.2f N, knocked_down: %d\n", ...
    result.force.peak_force, result.force.threshold, result.force.knocked_down);
disp("Finished random target simulation.");
