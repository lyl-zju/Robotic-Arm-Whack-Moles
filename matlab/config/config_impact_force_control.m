function cfg = config_impact_force_control()
% Parameters for the high-speed impact force-control experiment.

cfg.dt = 0.001;
cfg.total_time = 1.40;

cfg.approach_time = 0.22;
cfg.preimpact_clearance = 0.035;
cfg.impact_speed = 0.34;
cfg.retract_time = 0.30;

cfg.force_target_ratio = 1.45;
cfg.max_press_depth = 0.012;
cfg.force_control_timeout = 0.38;
cfg.contact_detect_force = 0.5;

cfg.contact_model = "hunt_crossley";
cfg.contact_stiffness = 120000.0;  % N / m^exponent
cfg.contact_damping = 9000.0;      % N*s / m^(exponent + 1)
cfg.contact_exponent = 1.5;
cfg.max_contact_force = 90.0;

cfg.sensor_delay = 0.004;
cfg.force_filter_time_constant = 0.006;
cfg.force_noise_std = 0.05;
cfg.random_seed = 7;

cfg.admittance_gain = 0.0014;  % m / (N*s)
cfg.max_force_error = 35.0;
cfg.force_release_margin = 0.004;

cfg.k_xyz_free = [850, 850, 1150];
cfg.d_xyz_free = [70, 70, 58];
cfg.k_xyz_contact = [900, 900, 650];
cfg.d_xyz_contact = [80, 80, 82];
cfg.k_xyz_retract = [750, 750, 900];
cfg.d_xyz_retract = [70, 70, 70];

cfg.posture_stiffness = [12, 10, 8, 5, 4, 3];
cfg.posture_damping = [5, 4, 3, 2, 1.5, 1.2];
cfg.joint_viscous_damping = [0.6, 0.6, 0.45, 0.25, 0.18, 0.12];

cfg.max_joint_torque = [150, 150, 120, 70, 45, 35];
cfg.max_joint_speed = [2.4, 2.4, 2.8, 3.2, 3.6, 3.6];
cfg.max_task_force = 160.0;
end
