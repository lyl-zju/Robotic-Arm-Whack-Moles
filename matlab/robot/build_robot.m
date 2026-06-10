function robot = build_robot(cfg)
if exist("loadrobot", "file") ~= 2
    error("Robotics System Toolbox is required for loadrobot().");
end

try
    robot = loadrobot(cfg.robot_name, "DataFormat", cfg.data_format);
catch
    error("Failed to load %s. Check Robotics System Toolbox robot model support.", cfg.robot_name);
end

if ~any(strcmp(robot.BodyNames, cfg.end_effector))
    warning("End effector %s not found. Using last body: %s", cfg.end_effector, robot.BodyNames{end});
    cfg.end_effector = string(robot.BodyNames{end});
end
end

