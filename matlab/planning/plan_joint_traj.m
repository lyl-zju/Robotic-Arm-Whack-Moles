function traj = plan_joint_traj(qWaypoints, segmentTime, dt)
numSegments = size(qWaypoints, 1) - 1;
numJoints = size(qWaypoints, 2);

tAll = [];
qAll = [];
qdAll = [];
qddAll = [];

for s = 1:numSegments
    localT = 0:dt:segmentTime;
    if s > 1
        localT = localT(2:end);
    end
    r = localT / segmentTime;
    blend = 10*r.^3 - 15*r.^4 + 6*r.^5;
    blendD = (30*r.^2 - 60*r.^3 + 30*r.^4) / segmentTime;
    blendDD = (60*r - 180*r.^2 + 120*r.^3) / (segmentTime^2);

    q0 = qWaypoints(s, :);
    q1 = qWaypoints(s + 1, :);
    dq = q1 - q0;

    qSeg = q0 + blend(:) .* dq;
    qdSeg = blendD(:) .* dq;
    qddSeg = blendDD(:) .* dq;

    tOffset = (s - 1) * segmentTime;
    tAll = [tAll; localT(:) + tOffset]; %#ok<AGROW>
    qAll = [qAll; qSeg]; %#ok<AGROW>
    qdAll = [qdAll; qdSeg]; %#ok<AGROW>
    qddAll = [qddAll; qddSeg]; %#ok<AGROW>
end

traj.t = tAll;
traj.q_des = qAll;
traj.qd_des = qdAll;
traj.qdd_des = qddAll;
traj.num_joints = numJoints;
end

