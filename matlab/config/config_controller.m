function cfg = config_controller()
cfg.type = "PD";
cfg.Kp = 80;
cfg.Kd = 18;
cfg.dt = 0.01;
cfg.segment_time = 0.65;
cfg.path_type = "straight";
end

