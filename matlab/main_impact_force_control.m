clc; close all;

rootDir = fileparts(fileparts(mfilename("fullpath")));
addpath(genpath(fullfile(rootDir, "matlab")));

cfgRobot = config_robot();
cfgBoard = config_board();
cfgForce = config_force();
cfgImpact = config_impact_force_control();

robot = build_robot(cfgRobot);
target = generate_random_target(cfgBoard);

fprintf("High-speed impact target id: %d, world = [%.3f %.3f %.3f]\n", ...
    target.id, target.position_world(1), target.position_world(2), target.position_world(3));

result = simulate_impact_force_control(robot, target.position_world, cfgRobot, cfgBoard, cfgForce, cfgImpact);
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

figure("Name", "Impact Target Board", "Color", "w");
draw_board(cfgBoard, target.position_board);
title(sprintf("Impact target %d on whac-a-mole board", target.id));

animate_robot(robot, result.traj, cfgRobot, cfgBoard, target.position_world, ...
    fullfile(videoDir, "impact_force_control_animation.gif"), result);

plot_impact_force_control(result, fullfile(figDir, "impact_force_control_results.png"));

save_result_json(result, fullfile(rootDir, "shared", "impact_force_control_result.json"));
fprintf("Peak contact force: %.2f N, threshold: %.2f N, knocked_down: %d\n", ...
    result.force.peak_force, result.force.threshold, result.force.knocked_down);
disp("Finished high-speed impact force-control simulation.");
