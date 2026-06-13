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

targetFile = fullfile(rootDir, "shared", "target.json");
target = read_target_json(targetFile);

if ~target.valid
    warning("No valid visual target in %s. Stop without hitting.", targetFile);
    return;
end

if isfield(target, "world") && ~isempty(target.world)
    targetWorld = target.world(:).';
else
    targetWorld = board_to_world(target.board(:).', cfgBoard);
end

fprintf("Vision target source=%s, world=[%.3f %.3f %.3f]\n", ...
    target.source, targetWorld(1), targetWorld(2), targetWorld(3));
fprintf("Hit motion mode: %s\n", cfgController.motion_mode);

result = execute_hit_target(robot, targetWorld, cfgRobot, cfgBoard, cfgController, cfgForce);
result.target_mode = target.source;
result.vision_timestamp = target.timestamp;

figDir = fullfile(rootDir, "results", "figures");
if ~exist(figDir, "dir")
    mkdir(figDir);
end
plot_results(result, fullfile(figDir, "vision_target_tracking.png"));

save_result_json(result, fullfile(rootDir, "shared", "result.json"));
export_q_traj(result.traj, fullfile(rootDir, "shared", "q_traj.csv"));

fprintf("Peak contact force: %.2f N, threshold: %.2f N, knocked_down: %d\n", ...
    result.force.peak_force, result.force.threshold, result.force.knocked_down);
disp("Finished vision-driven hit simulation.");
