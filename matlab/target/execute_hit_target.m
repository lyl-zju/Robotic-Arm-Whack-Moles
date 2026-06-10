function result = execute_hit_target(robot, targetWorld, cfgRobot, cfgBoard, cfgController)
timer = tic;
keyposes = make_hit_keyposes(targetWorld, cfgBoard);

q0 = cfgRobot.home_configuration;
qHover = solve_ik(robot, keyposes.hover, cfgRobot, q0);
qHit = solve_ik(robot, keyposes.hit, cfgRobot, qHover);
qBack = solve_ik(robot, keyposes.back, cfgRobot, qHit);

qWaypoints = [q0; qHover; qHit; qBack];
traj = plan_joint_traj(qWaypoints, cfgController.segment_time, cfgController.dt);
tracking = simulate_joint_tracking(traj, cfgController);

hitPosition = keyposes.hit;
hitError = norm(hitPosition(1:2) - targetWorld(1:2));

result.hit_success = hitError < cfgBoard.hit_threshold;
result.hit_error = hitError;
result.response_time = toc(timer);
result.controller = cfgController.type;
result.path_type = cfgController.path_type;
result.target_world = targetWorld;
result.keyposes = keyposes;
result.traj = traj;
result.tracking = tracking;
end

