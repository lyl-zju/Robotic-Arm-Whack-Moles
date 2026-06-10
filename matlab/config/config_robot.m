function cfg = config_robot()
cfg.robot_name = "universalUR5";
cfg.end_effector = "tool0";
cfg.data_format = "row";
cfg.home_configuration = [0, -pi/3, pi/2, -pi/2, -pi/2, 0];
cfg.ik_weights = [0.2, 0.2, 0.2, 1, 1, 1];
cfg.fixed_tool_eul_zyx = [0, pi, 0];
end

