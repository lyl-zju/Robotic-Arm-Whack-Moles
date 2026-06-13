function force = simulate_contact_force(robot, traj, tracking, cfgRobot, cfgBoard, cfgForce, targetWorld)
t = traj.t(:);
qActual = tracking.q_actual;
numSamples = numel(t);

eePosition = zeros(numSamples, 3);
for i = 1:numSamples
    pose = getTransform(robot, qActual(i, :), cfgRobot.end_effector);
    eePosition(i, :) = tform2trvec(pose);
end

eeVelocity = zeros(size(eePosition));
if numSamples > 1
    dtSamples = diff(t);
    eeVelocity(2:end, :) = diff(eePosition, 1, 1) ./ dtSamples;
    eeVelocity(1, :) = eeVelocity(2, :);
    dt = mean(dtSamples);
else
    dt = cfgControllerDtFallback();
end

targetWorld = targetWorld(:).';
xyError = vecnorm(eePosition(:, 1:2) - targetWorld(1:2), 2, 2);
xyInTarget = xyError <= cfgBoard.hit_threshold;

penetration = max(0, cfgBoard.z_hit - eePosition(:, 3));
closingVelocity = max(0, -eeVelocity(:, 3));
inContact = xyInTarget & penetration > 0;

normalForce = zeros(numSamples, 1);
normalForce(inContact) = cfgForce.k_contact .* penetration(inContact) + ...
    cfgForce.c_contact .* closingVelocity(inContact);
normalForce = min(normalForce, cfgForce.max_force);

aboveThreshold = normalForce >= cfgForce.threshold;
contactDuration = sum(inContact) * dt;
thresholdDuration = sum(aboveThreshold) * dt;

force.t = t;
force.normal = normalForce;
force.ee_position = eePosition;
force.ee_velocity = eeVelocity;
force.penetration = penetration;
force.xy_error = xyError;
force.xy_in_target = xyInTarget;
force.in_contact = inContact;
force.above_threshold = aboveThreshold;
force.peak_force = max(normalForce);
force.max_penetration = max(penetration);
force.min_xy_error = min(xyError);
force.contact_duration = contactDuration;
force.threshold_duration = thresholdDuration;
force.threshold = cfgForce.threshold;
force.knocked_down = force.peak_force >= cfgForce.threshold && ...
    thresholdDuration >= cfgForce.min_contact_time;
end

function dt = cfgControllerDtFallback()
dt = 0.01;
end
