function traj = plan_joint_traj(qWaypoints, segmentTime, dt, qdWaypoints, qddWaypoints)
numSegments = size(qWaypoints, 1) - 1;
numJoints = size(qWaypoints, 2);

if nargin < 4 || isempty(qdWaypoints)
    qdWaypoints = zeros(size(qWaypoints));
end
if nargin < 5 || isempty(qddWaypoints)
    qddWaypoints = zeros(size(qWaypoints));
end

if ~isequal(size(qdWaypoints), size(qWaypoints))
    error("qdWaypoints must have the same size as qWaypoints.");
end
if ~isequal(size(qddWaypoints), size(qWaypoints))
    error("qddWaypoints must have the same size as qWaypoints.");
end

tAll = [];
qAll = [];
qdAll = [];
qddAll = [];

for s = 1:numSegments
    localT = 0:dt:segmentTime;
    if s > 1
        localT = localT(2:end);
    end
    q0 = qWaypoints(s, :);
    q1 = qWaypoints(s + 1, :);
    qd0 = qdWaypoints(s, :);
    qd1 = qdWaypoints(s + 1, :);
    qdd0 = qddWaypoints(s, :);
    qdd1 = qddWaypoints(s + 1, :);

    [qSeg, qdSeg, qddSeg] = quintic_segment(q0, qd0, qdd0, q1, qd1, qdd1, localT);


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
traj.q_waypoints = qWaypoints;
traj.qd_waypoints = qdWaypoints;
traj.qdd_waypoints = qddWaypoints;
end

function [qSeg, qdSeg, qddSeg] = quintic_segment(q0, qd0, qdd0, q1, qd1, qdd1, localT)
T = localT(end);
t = localT(:);

a0 = q0;
a1 = qd0;
a2 = 0.5 * qdd0;

A = [
    T^3,     T^4,      T^5;
    3*T^2,   4*T^3,    5*T^4;
    6*T,     12*T^2,   20*T^3
];
b = [
    q1 - (a0 + a1*T + a2*T^2);
    qd1 - (a1 + 2*a2*T);
    qdd1 - 2*a2
];

coeff = A \ b;
a3 = coeff(1, :);
a4 = coeff(2, :);
a5 = coeff(3, :);

qSeg = a0 + t .* a1 + t.^2 .* a2 + t.^3 .* a3 + t.^4 .* a4 + t.^5 .* a5;
qdSeg = a1 + 2*t .* a2 + 3*t.^2 .* a3 + 4*t.^3 .* a4 + 5*t.^4 .* a5;
qddSeg = 2*a2 + 6*t .* a3 + 12*t.^2 .* a4 + 20*t.^3 .* a5;
end

