function q = solve_ik(robot, pointWorld, cfgRobot, qSeed)
if nargin < 4 || isempty(qSeed)
    qSeed = cfgRobot.home_configuration;
end

ik = inverseKinematics("RigidBodyTree", robot);
targetPose = trvec2tform(pointWorld(:).') * eul2tform(cfgRobot.fixed_tool_eul_zyx, "ZYX");
[q, solInfo] = ik(cfgRobot.end_effector, targetPose, cfgRobot.ik_weights, qSeed);

if solInfo.ExitFlag <= 0
    warning("IK did not fully converge. Status: %s", solInfo.Status);
end
end

