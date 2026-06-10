clear; clc; close all;

rootDir = fileparts(fileparts(mfilename("fullpath")));
addpath(genpath(fullfile(rootDir, "matlab")));

fprintf("=== UR5 model loading test ===\n");
fprintf("Project root: %s\n", rootDir);

if exist("loadrobot", "file") ~= 2
    error("Robotics System Toolbox is not available: loadrobot() was not found.");
end
fprintf("[OK] loadrobot() is available.\n");

cfgRobot = config_robot();
fprintf("Robot model name: %s\n", cfgRobot.robot_name);
fprintf("Expected end effector: %s\n", cfgRobot.end_effector);
fprintf("Data format: %s\n", cfgRobot.data_format);

robot = build_robot(cfgRobot);

if ~isa(robot, "rigidBodyTree")
    error("build_robot() returned %s instead of rigidBodyTree.", class(robot));
end
fprintf("[OK] build_robot() returned a rigidBodyTree.\n");

if ~strcmp(string(robot.DataFormat), cfgRobot.data_format)
    error("Robot DataFormat is %s, expected %s.", string(robot.DataFormat), cfgRobot.data_format);
end
fprintf("[OK] DataFormat is %s.\n", robot.DataFormat);

bodyNames = string(robot.BodyNames);
if ~any(bodyNames == cfgRobot.end_effector)
    fprintf("Available bodies:\n");
    fprintf("  %s\n", bodyNames);
    error("End effector body %s was not found in the loaded UR5 model.", cfgRobot.end_effector);
end
fprintf("[OK] End effector body %s exists.\n", cfgRobot.end_effector);

homeQ = cfgRobot.home_configuration;
expectedDofs = numel(homeConfiguration(robot));
if numel(homeQ) ~= expectedDofs
    error("home_configuration has %d values, but the loaded robot expects %d DOF.", numel(homeQ), expectedDofs);
end
fprintf("[OK] home_configuration length matches robot DOF: %d.\n", expectedDofs);

eePose = getTransform(robot, homeQ, cfgRobot.end_effector);
eePosition = tform2trvec(eePose);
fprintf("Home end-effector position: [%.4f %.4f %.4f] m\n", eePosition(1), eePosition(2), eePosition(3));

figure("Name", "UR5 load test", "Color", "w");
show(robot, homeQ, "Frames", "on", "Visuals", "on");
axis equal;
view(135, 25);
title("UR5 loaded successfully");

fprintf("[PASS] UR5 model loaded successfully.\n");
fprintf("A figure window should now show the UR5 robot at home_configuration.\n");
