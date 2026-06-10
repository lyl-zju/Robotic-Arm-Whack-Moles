function report = check_reachability(robot, cfgRobot, cfgBoard)
numTargets = size(cfgBoard.holes_world, 1);
report.reachable = false(numTargets, 1);
report.q = cell(numTargets, 1);

for i = 1:numTargets
    try
        q = solve_ik(robot, cfgBoard.holes_world(i, :), cfgRobot, cfgRobot.home_configuration);
        report.reachable(i) = all(isfinite(q));
        report.q{i} = q;
    catch err
        report.reachable(i) = false;
        report.q{i} = [];
        warning("Target %d is not reachable: %s", i, err.message);
    end
end
end

