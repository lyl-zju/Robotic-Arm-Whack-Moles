function cfg = config_controller()
cfg.type = "PD";
cfg.Kp = 120;
cfg.Kd = 24;
cfg.dt = 0.01;
cfg.segment_time = 0.35;
cfg.path_type = "straight";
cfg.motion_mode = normalize_motion_mode(getenv("HIT_MOTION_MODE"));
end
