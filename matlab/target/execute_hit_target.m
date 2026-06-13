function result = execute_hit_target(robot, targetWorld, cfgRobot, cfgBoard, cfgController, cfgForce)
if nargin < 6 || isempty(cfgForce)
    cfgForce = config_force();
end
if ~isfield(cfgController, "motion_mode")
    cfgController.motion_mode = normalize_motion_mode("");
end

timer = tic;
keyposes = make_hit_keyposes(targetWorld, cfgBoard, cfgForce);

q0 = cfgRobot.home_configuration;
qHover = solve_ik(robot, keyposes.hover, cfgRobot, q0);
qContact = solve_ik(robot, keyposes.contact, cfgRobot, qHover);
qPress = solve_ik(robot, keyposes.press, cfgRobot, qContact);
qBack = solve_ik(robot, keyposes.back, cfgRobot, qPress);

qWaypoints = [q0; qHover; qContact; qPress; qBack];
hoverWaypointIndex = 2;
[qdWaypoints, qddWaypoints] = make_hit_motion_boundaries( ...
    qWaypoints, cfgController, zeros(size(q0)), hoverWaypointIndex);
traj = plan_joint_traj(qWaypoints, cfgController.segment_time, cfgController.dt, ...
    qdWaypoints, qddWaypoints);
tracking = simulate_joint_tracking(traj, cfgController);
force = simulate_contact_force(robot, traj, tracking, cfgRobot, cfgBoard, cfgForce, targetWorld);

hitError = force.min_xy_error;

result.hit_success = force.knocked_down;
result.hit_error = hitError;
result.response_time = toc(timer);
result.controller = cfgController.type;
result.path_type = cfgController.path_type;
result.motion_mode = normalize_motion_mode(cfgController.motion_mode);
result.target_world = targetWorld;
result.keyposes = keyposes;
result.traj = traj;
result.tracking = tracking;
result.force = force;
end
