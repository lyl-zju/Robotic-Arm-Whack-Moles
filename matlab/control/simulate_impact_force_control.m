function result = simulate_impact_force_control(robot, targetWorld, cfgRobot, cfgBoard, cfgForce, cfgImpact)
% Simulate high-speed impact, contact dynamics, and post-impact force control.

if nargin < 6 || isempty(cfgImpact)
    cfgImpact = config_impact_force_control();
end
if nargin < 5 || isempty(cfgForce)
    cfgForce = config_force();
end

rng(cfgImpact.random_seed);

targetWorld = targetWorld(:).';
qHover = solve_ik(robot, [targetWorld(1), targetWorld(2), cfgBoard.z_hover], ...
    cfgRobot, cfgRobot.home_configuration);

numJoints = numel(qHover);
q = qHover(:);
qd = zeros(numJoints, 1);
qNominal = qHover(:);

dt = cfgImpact.dt;
t = (0:dt:cfgImpact.total_time).';
numSamples = numel(t);

qLog = zeros(numSamples, numJoints);
qdLog = zeros(numSamples, numJoints);
qddLog = zeros(numSamples, numJoints);
xLog = zeros(numSamples, 3);
xdLog = zeros(numSamples, 3);
xDesLog = zeros(numSamples, 3);
xdDesLog = zeros(numSamples, 3);
forceRawLog = zeros(numSamples, 1);
forceMeasuredLog = zeros(numSamples, 1);
forceFilteredLog = zeros(numSamples, 1);
forceDesiredLog = zeros(numSamples, 1);
penetrationLog = zeros(numSamples, 1);
closingVelocityLog = zeros(numSamples, 1);
tauLog = zeros(numSamples, numJoints);
phaseLog = strings(numSamples, 1);

forceDesired = cfgForce.threshold * cfgImpact.force_target_ratio;
forceFiltered = 0.0;
delaySamples = max(0, round(cfgImpact.sensor_delay / dt));
forceDelayBuffer = zeros(delaySamples + 1, 1);
filterAlpha = dt / max(cfgImpact.force_filter_time_constant + dt, eps);

phase = "approach";
firstContactTime = NaN;
successTime = NaN;
thresholdDuration = 0.0;
contactDuration = 0.0;
forcePhaseStart = NaN;
retractStartTime = NaN;
retractStartZ = cfgBoard.z_hover;
zForceCmd = cfgBoard.z_hit;
prevZDes = cfgBoard.z_hover;

for k = 1:numSamples
    tk = t(k);
    pose = getTransform(robot, q.', cfgRobot.end_effector);
    x = tform2trvec(pose).';
    J = geometricJacobian(robot, q.', cfgRobot.end_effector);
    Jv = J(4:6, :);
    xd = Jv * qd;

    [forceRaw, penetration, closingVelocity] = contact_force_model( ...
        x(3), xd(3), cfgBoard, cfgImpact);
    forceDelayBuffer = [forceDelayBuffer(2:end); forceRaw]; %#ok<AGROW>
    forceMeasured = forceDelayBuffer(1);
    if cfgImpact.force_noise_std > 0
        forceMeasured = max(0.0, forceMeasured + cfgImpact.force_noise_std * randn());
    end
    forceFiltered = forceFiltered + filterAlpha * (forceMeasured - forceFiltered);

    contactDetected = forceRaw >= cfgImpact.contact_detect_force || penetration > 0;
    if contactDetected && isnan(firstContactTime)
        firstContactTime = tk;
        phase = "force";
        forcePhaseStart = tk;
        zForceCmd = min(x(3), cfgBoard.z_hit + cfgImpact.force_release_margin);
        prevZDes = zForceCmd;
    end

    if forceRaw > 0
        contactDuration = contactDuration + dt;
    end
    if forceRaw >= cfgForce.threshold
        thresholdDuration = thresholdDuration + dt;
        if isnan(successTime) && thresholdDuration >= cfgForce.min_contact_time
            successTime = tk;
        end
    end

    if phase == "approach"
        [zDes, zdDes] = approach_reference(tk, cfgBoard, cfgImpact);
    elseif phase == "force"
        forceError = clamp_scalar(forceDesired - forceFiltered, ...
            -cfgImpact.max_force_error, cfgImpact.max_force_error);
        zForceCmd = zForceCmd - cfgImpact.admittance_gain * forceError * dt;
        zUpper = cfgBoard.z_hit + cfgImpact.force_release_margin;
        zLower = cfgBoard.z_hit - cfgImpact.max_press_depth;
        zForceCmd = clamp_scalar(zForceCmd, zLower, zUpper);
        zDes = zForceCmd;
        zdDes = (zDes - prevZDes) / dt;

        forceTimedOut = ~isnan(forcePhaseStart) && ...
            (tk - forcePhaseStart >= cfgImpact.force_control_timeout);
        if (~isnan(successTime) && tk - successTime >= cfgForce.min_contact_time) || forceTimedOut
            phase = "retract";
            retractStartTime = tk;
            retractStartZ = zDes;
        end
    elseif phase == "retract"
        [zDes, zdDes] = retract_reference(tk, retractStartTime, retractStartZ, ...
            cfgBoard.z_hover, cfgImpact);
    else
        zDes = cfgBoard.z_hover;
        zdDes = 0.0;
    end
    prevZDes = zDes;

    xDes = [targetWorld(1); targetWorld(2); zDes];
    xdDes = [0.0; 0.0; zdDes];
    [kXyz, dXyz] = impedance_gains_for_phase(phase, cfgImpact);

    fTask = kXyz(:) .* (xDes - x) + dXyz(:) .* (xdDes - xd);
    fTask = clamp_vector_norm(fTask, cfgImpact.max_task_force);

    tauTask = Jv.' * fTask;
    tauPosture = cfgImpact.posture_stiffness(:) .* (qNominal - q) - ...
        cfgImpact.posture_damping(:) .* qd;
    tauGravity = gravityTorque(robot, q.').';
    tauCommand = tauTask + tauPosture + tauGravity;
    tauCommand = clamp_vector(tauCommand, cfgImpact.max_joint_torque(:));

    contactWrench = [0; 0; 0; 0; 0; forceRaw];
    tauContact = J.' * contactWrench;

    mass = massMatrix(robot, q.');
    coriolis = velocityProduct(robot, q.', qd.').';
    tauDamping = cfgImpact.joint_viscous_damping(:) .* qd;
    qdd = mass \ (tauCommand + tauContact - coriolis - tauGravity - tauDamping);

    qd = qd + qdd * dt;
    qd = clamp_vector(qd, cfgImpact.max_joint_speed(:));
    q = q + qd * dt;

    qLog(k, :) = q.';
    qdLog(k, :) = qd.';
    qddLog(k, :) = qdd.';
    xLog(k, :) = x.';
    xdLog(k, :) = xd.';
    xDesLog(k, :) = xDes.';
    xdDesLog(k, :) = xdDes.';
    forceRawLog(k) = forceRaw;
    forceMeasuredLog(k) = forceMeasured;
    forceFilteredLog(k) = forceFiltered;
    forceDesiredLog(k) = forceDesired;
    penetrationLog(k) = penetration;
    closingVelocityLog(k) = closingVelocity;
    tauLog(k, :) = tauCommand.';
    phaseLog(k) = phase;
end

xyError = vecnorm(xLog(:, 1:2) - targetWorld(1:2), 2, 2);
aboveThreshold = forceRawLog >= cfgForce.threshold;
inContact = forceRawLog > 0;

trackingError = xDesLog - xLog;
trackingRmse = sqrt(mean(trackingError(:).^2));
trackingMax = max(abs(trackingError(:)));

result.hit_success = max(forceRawLog) >= cfgForce.threshold && ...
    thresholdDuration >= cfgForce.min_contact_time;
result.hit_error = min(xyError);
result.response_time = first_finite_or_default(successTime, t(end));
result.controller = "operational_space_impedance_admittance_force_control";
result.path_type = "high_speed_impact";
result.motion_mode = "impact_force_control";
result.target_world = targetWorld;

result.traj.t = t;
result.traj.q_des = qLog;
result.traj.qd_des = qdLog;
result.traj.qdd_des = qddLog;
result.traj.x_des = xDesLog;
result.traj.xd_des = xdDesLog;

result.tracking.q_actual = qLog;
result.tracking.qd_actual = qdLog;
result.tracking.error = trackingError;
result.tracking.rmse = trackingRmse;
result.tracking.max_abs_error = trackingMax;

result.ee.position = xLog;
result.ee.velocity = xdLog;

result.force.t = t;
result.force.normal = forceRawLog;
result.force.measured = forceMeasuredLog;
result.force.filtered = forceFilteredLog;
result.force.desired = forceDesiredLog;
result.force.penetration = penetrationLog;
result.force.closing_velocity = closingVelocityLog;
result.force.xy_error = xyError;
result.force.xy_in_target = xyError <= cfgBoard.hit_threshold;
result.force.in_contact = inContact;
result.force.above_threshold = aboveThreshold;
result.force.peak_force = max(forceRawLog);
result.force.threshold = cfgForce.threshold;
result.force.target = forceDesired;
result.force.max_penetration = max(penetrationLog);
result.force.min_xy_error = min(xyError);
result.force.contact_duration = contactDuration;
result.force.threshold_duration = thresholdDuration;
result.force.knocked_down = result.hit_success;
result.force.first_contact_time = firstContactTime;
result.force.success_time = successTime;

result.control.phase = phaseLog;
result.control.torque = tauLog;
result.control.cfg = cfgImpact;
end

function [zDes, zdDes] = approach_reference(t, cfgBoard, cfgImpact)
z0 = cfgBoard.z_hover;
z1 = cfgBoard.z_hit + cfgImpact.preimpact_clearance;
if t <= cfgImpact.approach_time
    u = clamp_scalar(t / cfgImpact.approach_time, 0.0, 1.0);
    [s, sd] = smoothstep_with_derivative(u);
    zDes = z0 + (z1 - z0) * s;
    zdDes = (z1 - z0) * sd / cfgImpact.approach_time;
else
    driveTime = t - cfgImpact.approach_time;
    zDes = z1 - cfgImpact.impact_speed * driveTime;
    zdDes = -cfgImpact.impact_speed;
    zDes = max(zDes, cfgBoard.z_hit - cfgImpact.max_press_depth);
end
end

function [zDes, zdDes] = retract_reference(t, t0, z0, z1, cfgImpact)
if isnan(t0)
    zDes = z1;
    zdDes = 0.0;
    return;
end
u = clamp_scalar((t - t0) / cfgImpact.retract_time, 0.0, 1.0);
[s, sd] = smoothstep_with_derivative(u);
zDes = z0 + (z1 - z0) * s;
zdDes = (z1 - z0) * sd / cfgImpact.retract_time;
end

function [s, sd] = smoothstep_with_derivative(u)
s = u * u * (3 - 2 * u);
sd = 6 * u * (1 - u);
end

function [force, penetration, closingVelocity] = contact_force_model(z, zd, cfgBoard, cfgImpact)
penetration = max(0.0, cfgBoard.z_hit - z);
closingVelocity = max(0.0, -zd);
if penetration <= 0
    force = 0.0;
    return;
end

if cfgImpact.contact_model == "hunt_crossley"
    p = penetration ^ cfgImpact.contact_exponent;
    force = cfgImpact.contact_stiffness * p + cfgImpact.contact_damping * p * closingVelocity;
else
    force = cfgImpact.contact_stiffness * penetration + ...
        cfgImpact.contact_damping * closingVelocity;
end
force = clamp_scalar(force, 0.0, cfgImpact.max_contact_force);
end

function [kXyz, dXyz] = impedance_gains_for_phase(phase, cfgImpact)
if phase == "force"
    kXyz = cfgImpact.k_xyz_contact;
    dXyz = cfgImpact.d_xyz_contact;
elseif phase == "retract"
    kXyz = cfgImpact.k_xyz_retract;
    dXyz = cfgImpact.d_xyz_retract;
else
    kXyz = cfgImpact.k_xyz_free;
    dXyz = cfgImpact.d_xyz_free;
end
end

function x = clamp_scalar(x, lower, upper)
x = min(max(x, lower), upper);
end

function v = clamp_vector(v, limits)
limits = abs(limits(:));
v = min(max(v, -limits), limits);
end

function v = clamp_vector_norm(v, maxNorm)
n = norm(v);
if n > maxNorm
    v = v * (maxNorm / n);
end
end

function value = first_finite_or_default(value, fallback)
if ~isfinite(value)
    value = fallback;
end
end
