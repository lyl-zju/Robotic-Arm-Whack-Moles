requestedHitMotionMode = "";
if exist("hitMotionMode", "var")
    requestedHitMotionMode = hitMotionMode;
elseif exist("motionMode", "var")
    requestedHitMotionMode = motionMode;
end
clearvars -except requestedHitMotionMode; clc; close all;

rootDir = fileparts(fileparts(mfilename("fullpath")));
addpath(genpath(fullfile(rootDir, "matlab")));

cfgRobot = config_robot();
cfgBoard = config_board();
cfgForce = config_force();
cfgImpact = config_impact_force_control();
cfgController = config_controller();

if strlength(string(requestedHitMotionMode)) > 0
    cfgController = apply_motion_mode_override(cfgController, requestedHitMotionMode);
elseif strlength(string(getenv("HIT_MOTION_MODE"))) == 0
    cfgController.motion_mode = "continuous";
else
    cfgController.motion_mode = normalize_motion_mode(cfgController.motion_mode);
end

dt = cfgImpact.dt;
cfgController.dt = dt;
cfgController.path_type = "direct";

rng(cfgImpact.random_seed);
robot = build_robot(cfgRobot);

numJoints = numel(cfgRobot.home_configuration);
qAct = cfgRobot.home_configuration(:).';
qdAct = zeros(1, numJoints);
holdQ = qAct;

activeTraj = [];
tTraj = 0;
currentTargetBoard = [];
currentTargetWorld = [];
currentKeyposes = [];
currentPlannedPathWorld = [];
eeTrail = zeros(0, 3);

forceState = init_force_state(cfgForce, cfgImpact, cfgBoard, dt);

tCurrent = 0;
stepIndex = 0;
graphicsEvery = max(1, round(0.04 / dt));

[boardFig, boardAx, targetMarker, eeMarker] = create_board_window(cfgBoard);
[robotFig, robotAx, forceHud] = create_robot_window();

show_robot_frame(robotAx, robot, qAct, cfgBoard, currentTargetWorld, ...
    currentKeyposes, currentPlannedPathWorld, eeTrail, tCurrent, ...
    force_visual_state(forceState, tCurrent), forceHud);

fprintf("Interactive impact force-control simulation started.\n");
fprintf("Motion mode: %s\n", cfgController.motion_mode);
fprintf("Click a target on the 2D board. New clicks preempt the current motion.\n");
fprintf("Press Esc in the board window, or close a figure, to stop.\n");

while is_simulation_running(boardFig, robotFig)
    stepTimer = tic;
    drawnow limitrate;

    if getappdata(boardFig, "is_new_target")
        setappdata(boardFig, "is_new_target", false);

        targetBoard = getappdata(boardFig, "target_board");
        targetWorld = getappdata(boardFig, "target_world");
        qStart = qAct;
        qdStart = qdAct;

        try
            [newTraj, planInfo] = plan_direct_preempted_impact_traj( ...
                robot, targetWorld, cfgRobot, cfgBoard, cfgController, ...
                cfgForce, qStart, qdStart);

            activeTraj = newTraj;
            tTraj = 0;
            holdQ = activeTraj.q_des(end, :);
            currentTargetBoard = targetBoard;
            currentTargetWorld = targetWorld;
            currentKeyposes = planInfo.keyposes;
            currentPlannedPathWorld = planInfo.path_world;
            forceState = reset_force_state(forceState, cfgForce, cfgImpact, ...
                cfgBoard, dt, tCurrent, currentTargetWorld);

            title(boardAx, sprintf("target=[%.3f %.3f], direct replanned from current state", ...
                targetBoard(1), targetBoard(2)), "Color", [0.0 0.45 0.15]);

            fprintf("t=%.2f s: direct preempt to target board=[%.3f %.3f], waypoints=%d\n", ...
                tCurrent, currentTargetBoard(1), currentTargetBoard(2), ...
                size(activeTraj.q_waypoints, 1));
        catch err
            warning("main_interactive_impact_force_control:ReplanFailed", ...
                "Direct replan failed. Keeping current motion. Detail: %s", err.message);
            title(boardAx, "direct replan failed; keeping current motion", ...
                "Color", [0.80 0.05 0.05]);
        end
    end

    if isempty(activeTraj)
        qRef = holdQ;
        qdRef = zeros(1, numJoints);
        qddRef = zeros(1, numJoints);
        nominalPhase = "idle";
    else
        [qRef, qdRef, qddRef, isTrajDone] = sample_joint_traj(activeTraj, tTraj);
        nominalPhase = trajectory_phase(activeTraj, tTraj);
        if isTrajDone
            holdQ = activeTraj.q_des(end, :);
            activeTraj = [];
            tTraj = 0;
        else
            tTraj = tTraj + dt;
        end
    end

    [qAct, qdAct, ~, forceState, eePosition] = step_impact_force_control( ...
        robot, qAct, qdAct, qRef, qdRef, qddRef, currentTargetWorld, ...
        nominalPhase, cfgRobot, cfgBoard, cfgForce, cfgImpact, forceState, ...
        dt, tCurrent);

    if mod(stepIndex, graphicsEvery) == 0
        eeTrail = [eeTrail; eePosition(:).']; %#ok<AGROW>
        if size(eeTrail, 1) > 600
            eeTrail = eeTrail(end-599:end, :);
        end

        update_board_ee_marker(eeMarker, eePosition, cfgBoard);
        show_robot_frame(robotAx, robot, qAct, cfgBoard, currentTargetWorld, ...
            currentKeyposes, currentPlannedPathWorld, eeTrail, tCurrent, ...
            force_visual_state(forceState, tCurrent), forceHud);
    end

    tCurrent = tCurrent + dt;
    stepIndex = stepIndex + 1;
    pause(max(0, dt - toc(stepTimer)));
end

fprintf("Interactive impact force-control simulation stopped at t=%.2f s.\n", tCurrent);

function [fig, ax, targetMarker, eeMarker] = create_board_window(cfgBoard)
fig = figure("Name", "Interactive Impact Force-Control Target Board", ...
    "Color", "w", "NumberTitle", "off");
ax = axes("Parent", fig);
axes(ax);

draw_board(cfgBoard, []);
title(ax, "click a target; new clicks immediately replan from current arm state", ...
    "Color", [0.10 0.10 0.10]);

hold(ax, "on");
targetMarker = plot(ax, NaN, NaN, "ro", "MarkerSize", 16, "LineWidth", 2.5);
eeMarker = plot(ax, NaN, NaN, "bo", "MarkerSize", 9, "LineWidth", 1.5);
legend(ax, [targetMarker, eeMarker], ...
    {"current target", "end effector"}, "Location", "southoutside");

setappdata(fig, "target_board", []);
setappdata(fig, "target_world", []);
setappdata(fig, "is_new_target", false);
setappdata(fig, "is_running", true);

set(fig, "WindowButtonDownFcn", ...
    @(src, event)on_board_click(src, event, cfgBoard, ax, targetMarker));
set(fig, "KeyPressFcn", @(src, event)on_key_press(src, event));
end

function [fig, ax, hud] = create_robot_window()
fig = figure("Name", "Interactive Impact Force-Control Robot", ...
    "Color", "w", "NumberTitle", "off");
ax = axes("Parent", fig);
hud = create_force_hud(fig);
end

function on_board_click(fig, ~, cfgBoard, ax, targetMarker)
if ~strcmp(get(fig, "SelectionType"), "normal")
    return;
end

point = get(ax, "CurrentPoint");
clickedPoint = point(1, 1:2);
if ~is_inside_board(clickedPoint, cfgBoard)
    title(ax, "target is outside the board", "Color", [0.75 0.05 0.05]);
    drawnow limitrate;
    return;
end

targetWorld = board_to_world(clickedPoint, cfgBoard);
setappdata(fig, "target_board", clickedPoint);
setappdata(fig, "target_world", targetWorld);
setappdata(fig, "is_new_target", true);

set(targetMarker, "XData", clickedPoint(1), "YData", clickedPoint(2));
title(ax, "new target accepted; direct replanning now", "Color", [0.0 0.48 0.18]);
drawnow limitrate;
end

function on_key_press(fig, event)
if strcmp(event.Key, "escape")
    setappdata(fig, "is_running", false);
end
end

function inside = is_inside_board(pointBoard, cfgBoard)
x = pointBoard(1);
y = pointBoard(2);
inside = x >= -cfgBoard.width / 2 && x <= cfgBoard.width / 2 && ...
    y >= -cfgBoard.height / 2 && y <= cfgBoard.height / 2;
end

function running = is_simulation_running(boardFig, robotFig)
running = isgraphics(boardFig) && isgraphics(robotFig);
if running && isappdata(boardFig, "is_running")
    running = logical(getappdata(boardFig, "is_running"));
end
end

function [traj, planInfo] = plan_direct_preempted_impact_traj( ...
    robot, targetWorld, cfgRobot, cfgBoard, cfgController, cfgForce, qStart, qdStart)
targetWorld = targetWorld(:).';
keyposes = make_hit_keyposes(targetWorld, cfgBoard, cfgForce);

qSeed = qStart;
qHover = solve_ik(robot, keyposes.hover, cfgRobot, qSeed);
qContact = solve_ik(robot, keyposes.contact, cfgRobot, qHover);
qHit = solve_ik(robot, keyposes.hit, cfgRobot, qContact);
qBack = solve_ik(robot, keyposes.back, cfgRobot, qHit);
qWaypoints = [qStart; qHover; qContact; qHit; qBack];

hoverWaypointIndex = 2;
[qdWaypoints, qddWaypoints] = make_hit_motion_boundaries( ...
    qWaypoints, cfgController, qdStart, hoverWaypointIndex);

traj = plan_joint_traj(qWaypoints, cfgController.segment_time, cfgController.dt, ...
    qdWaypoints, qddWaypoints);
traj.path_type = "direct";
traj.motion_mode = cfgController.motion_mode;
traj.descent_start_time = max(0, (hoverWaypointIndex - 1) * cfgController.segment_time);
traj.retract_start_time = max(0, (size(qWaypoints, 1) - 2) * cfgController.segment_time);

startPose = getTransform(robot, qStart, cfgRobot.end_effector);
startPosition = tform2trvec(startPose);
planInfo.keyposes = keyposes;
planInfo.path_world = [
    startPosition;
    keyposes.hover;
    keyposes.contact;
    keyposes.hit;
    keyposes.back
];
end

function [qRef, qdRef, qddRef, done] = sample_joint_traj(traj, tQuery)
if tQuery >= traj.t(end)
    qRef = traj.q_des(end, :);
    qdRef = zeros(1, traj.num_joints);
    qddRef = zeros(1, traj.num_joints);
    done = true;
    return;
end

qRef = interp1(traj.t, traj.q_des, tQuery, "linear");
qdRef = interp1(traj.t, traj.qd_des, tQuery, "linear");
qddRef = interp1(traj.t, traj.qdd_des, tQuery, "linear");
done = false;
end

function phase = trajectory_phase(traj, tQuery)
if isempty(traj)
    phase = "idle";
elseif tQuery >= traj.retract_start_time
    phase = "retract";
elseif tQuery >= traj.descent_start_time
    phase = "impact";
else
    phase = "approach";
end
end

function state = init_force_state(cfgForce, cfgImpact, cfgBoard, dt)
state = struct();
state.phase = "idle";
state.target_world = [];
state.force_desired = cfgForce.threshold * cfgImpact.force_target_ratio;
state.threshold = cfgForce.threshold;
state.delay_samples = max(0, round(cfgImpact.sensor_delay / dt));
state.delay_buffer = zeros(state.delay_samples + 1, 1);
state.filter_alpha = dt / max(cfgImpact.force_filter_time_constant + dt, eps);
state.noise_std = cfgImpact.force_noise_std;
state.filtered_force = 0.0;
state.measured_force = 0.0;
state.raw_force = 0.0;
state.peak_force = 0.0;
state.penetration = 0.0;
state.closing_velocity = 0.0;
state.xy_error = inf;
state.xy_in_target = false;
state.contact_duration = 0.0;
state.threshold_duration = 0.0;
state.first_contact_time = NaN;
state.success_time = NaN;
state.force_phase_start = NaN;
state.z_cmd = cfgBoard.z_hit;
state.prev_z_cmd = cfgBoard.z_hover;
state.success = false;
state.replanned_time = NaN;
end

function state = reset_force_state(~, cfgForce, cfgImpact, cfgBoard, dt, tCurrent, targetWorld)
state = init_force_state(cfgForce, cfgImpact, cfgBoard, dt);
state.phase = "approach";
state.target_world = targetWorld(:).';
state.replanned_time = tCurrent;
end

function [qNext, qdNext, qdd, forceState, eePosition] = step_impact_force_control( ...
    robot, qAct, qdAct, qRef, qdRef, qddRef, targetWorld, nominalPhase, ...
    cfgRobot, cfgBoard, cfgForce, cfgImpact, forceState, dt, tCurrent)
q = qAct(:);
qd = qdAct(:);
qRefCol = qRef(:);
qdRefCol = qdRef(:);
qddRefCol = qddRef(:);

pose = getTransform(robot, q.', cfgRobot.end_effector);
eePosition = tform2trvec(pose).';
J = geometricJacobian(robot, q.', cfgRobot.end_effector);
Jv = J(4:6, :);
eeVelocity = Jv * qd;

[forceRaw, penetration, closingVelocity, xyError, xyInTarget] = contact_measurement( ...
    eePosition, eeVelocity, targetWorld, cfgBoard, cfgImpact);
forceState = update_force_measurement(forceState, forceRaw, penetration, ...
    closingVelocity, xyError, xyInTarget, cfgForce, dt, tCurrent);
forceState = update_force_phase(forceState, nominalPhase, eePosition, ...
    cfgBoard, cfgForce, cfgImpact, dt, tCurrent);

xRef = tform2trvec(getTransform(robot, qRefCol.', cfgRobot.end_effector)).';
xdRef = Jv * qdRefCol;

if isempty(targetWorld) || forceState.phase == "idle"
    xDes = xRef;
    xdDes = xdRef;
elseif forceState.phase == "force"
    forceError = clamp_scalar(forceState.force_desired - forceState.filtered_force, ...
        -cfgImpact.max_force_error, cfgImpact.max_force_error);
    zPrev = forceState.z_cmd;
    forceState.z_cmd = forceState.z_cmd - cfgImpact.admittance_gain * forceError * dt;
    zUpper = cfgBoard.z_hit + cfgImpact.force_release_margin;
    zLower = cfgBoard.z_hit - cfgImpact.max_press_depth;
    forceState.z_cmd = clamp_scalar(forceState.z_cmd, zLower, zUpper);

    xDes = [targetWorld(1); targetWorld(2); forceState.z_cmd];
    xdDes = [0.0; 0.0; (forceState.z_cmd - zPrev) / dt];
    forceState.prev_z_cmd = forceState.z_cmd;
else
    xDes = xRef;
    xdDes = xdRef;
end

[kXyz, dXyz] = impedance_gains_for_phase(forceState.phase, nominalPhase, cfgImpact);
fTask = kXyz(:) .* (xDes - eePosition) + dXyz(:) .* (xdDes - eeVelocity);
fTask = clamp_vector_norm(fTask, cfgImpact.max_task_force);

tauTask = Jv.' * fTask;
tauPosture = cfgImpact.posture_stiffness(:) .* (qRefCol - q) + ...
    cfgImpact.posture_damping(:) .* (qdRefCol - qd);
tauFeedforward = 0.15 * qddRefCol;
tauGravity = gravityTorque(robot, q.').';
tauCommand = tauTask + tauPosture + tauFeedforward + tauGravity;
tauCommand = clamp_vector(tauCommand, cfgImpact.max_joint_torque(:));

contactWrench = [0; 0; 0; 0; 0; forceRaw];
tauContact = J.' * contactWrench;

mass = massMatrix(robot, q.');
coriolis = velocityProduct(robot, q.', qd.').';
tauDamping = cfgImpact.joint_viscous_damping(:) .* qd;
qdd = mass \ (tauCommand + tauContact - coriolis - tauGravity - tauDamping);

qddLimit = 65;
qdd = clamp_vector(qdd, qddLimit * ones(numel(qdd), 1));
qdNextCol = qd + qdd * dt;
qdNextCol = clamp_vector(qdNextCol, cfgImpact.max_joint_speed(:));
qNextCol = q + qdNextCol * dt;

qNext = qNextCol.';
qdNext = qdNextCol.';
end

function [forceRaw, penetration, closingVelocity, xyError, xyInTarget] = contact_measurement( ...
    eePosition, eeVelocity, targetWorld, cfgBoard, cfgImpact)
if isempty(targetWorld)
    forceRaw = 0.0;
    penetration = 0.0;
    closingVelocity = 0.0;
    xyError = inf;
    xyInTarget = false;
    return;
end

xyError = norm(eePosition(1:2) - targetWorld(1:2).');
xyInTarget = xyError <= cfgBoard.hit_threshold;
if ~xyInTarget
    forceRaw = 0.0;
    penetration = 0.0;
    closingVelocity = 0.0;
    return;
end

penetration = max(0.0, cfgBoard.z_hit - eePosition(3));
closingVelocity = max(0.0, -eeVelocity(3));
if penetration <= 0
    forceRaw = 0.0;
    return;
end

if cfgImpact.contact_model == "hunt_crossley"
    p = penetration ^ cfgImpact.contact_exponent;
    forceRaw = cfgImpact.contact_stiffness * p + ...
        cfgImpact.contact_damping * p * closingVelocity;
else
    forceRaw = cfgImpact.contact_stiffness * penetration + ...
        cfgImpact.contact_damping * closingVelocity;
end
forceRaw = clamp_scalar(forceRaw, 0.0, cfgImpact.max_contact_force);
end

function state = update_force_measurement(state, forceRaw, penetration, ...
    closingVelocity, xyError, xyInTarget, cfgForce, dt, tCurrent)
state.delay_buffer = [state.delay_buffer(2:end); forceRaw];
forceMeasured = state.delay_buffer(1);
if state.noise_std > 0
    forceMeasured = max(0.0, forceMeasured + state.noise_std * randn());
end

state.raw_force = forceRaw;
state.measured_force = forceMeasured;
state.filtered_force = state.filtered_force + ...
    state.filter_alpha * (forceMeasured - state.filtered_force);
state.peak_force = max(state.peak_force, forceRaw);
state.penetration = penetration;
state.closing_velocity = closingVelocity;
state.xy_error = xyError;
state.xy_in_target = xyInTarget;

if forceRaw > 0
    state.contact_duration = state.contact_duration + dt;
else
    state.contact_duration = 0.0;
end

if forceRaw >= cfgForce.threshold
    state.threshold_duration = state.threshold_duration + dt;
else
    state.threshold_duration = 0.0;
end

if isnan(state.success_time) && state.threshold_duration >= cfgForce.min_contact_time
    state.success_time = tCurrent;
    state.success = true;
end
end

function state = update_force_phase(state, nominalPhase, eePosition, ...
    cfgBoard, cfgForce, cfgImpact, dt, tCurrent)
if state.phase == "idle"
    return;
end

contactDetected = state.raw_force >= cfgImpact.contact_detect_force || ...
    (state.xy_in_target && state.penetration > 0);

if contactDetected && isnan(state.first_contact_time)
    state.first_contact_time = tCurrent;
    state.force_phase_start = tCurrent;
    state.phase = "force";
    state.z_cmd = min(eePosition(3), cfgBoard.z_hit + cfgImpact.force_release_margin);
    state.prev_z_cmd = state.z_cmd;
end

if state.phase == "force"
    forceTimedOut = ~isnan(state.force_phase_start) && ...
        (tCurrent - state.force_phase_start >= cfgImpact.force_control_timeout);
    forceHeld = state.success && ...
        (tCurrent - state.success_time >= max(cfgForce.min_contact_time, dt));
    if forceHeld || forceTimedOut || nominalPhase == "retract"
        state.phase = "retract";
    end
elseif nominalPhase == "retract"
    state.phase = "retract";
elseif nominalPhase == "idle"
    state.phase = "idle";
else
    state.phase = "approach";
end
end

function [kXyz, dXyz] = impedance_gains_for_phase(forcePhase, nominalPhase, cfgImpact)
if forcePhase == "force"
    kXyz = cfgImpact.k_xyz_contact;
    dXyz = cfgImpact.d_xyz_contact;
elseif forcePhase == "retract" || nominalPhase == "retract"
    kXyz = cfgImpact.k_xyz_retract;
    dXyz = cfgImpact.d_xyz_retract;
else
    kXyz = cfgImpact.k_xyz_free;
    dXyz = cfgImpact.d_xyz_free;
end
end

function visual = force_visual_state(forceState, tCurrent)
visual.has_force = forceState.phase ~= "idle" || ~isempty(forceState.target_world);
visual.t = tCurrent;
visual.drop = max(0.0, forceState.penetration);
visual.force_ratio = 0.0;
visual.force = max(0.0, forceState.raw_force);
visual.filtered_force = max(0.0, forceState.filtered_force);
visual.peak_force = max(0.0, forceState.peak_force);
visual.threshold = forceState.threshold;
visual.target_force = forceState.force_desired;
visual.phase = forceState.phase;
visual.success = forceState.success;
if visual.threshold > 0
    visual.force_ratio = min(1.0, max(0.0, visual.force / visual.threshold));
end
if forceState.success && isfinite(forceState.success_time)
    u = min(1.0, (tCurrent - forceState.success_time) / 0.18);
    visual.drop = max(visual.drop, 0.032 * smoothstep(u));
end
end

function update_board_ee_marker(eeMarker, eePosition, cfgBoard)
if ~isgraphics(eeMarker)
    return;
end
pointBoard = world_to_board(eePosition, cfgBoard);
set(eeMarker, "XData", pointBoard(1), "YData", pointBoard(2));
end

function pointBoard = world_to_board(pointWorld, cfgBoard)
pointBoard3 = cfgBoard.R_world_board.' * (pointWorld(:) - cfgBoard.center_world(:));
pointBoard = pointBoard3(1:2).';
end

function show_robot_frame(ax, robot, q, cfgBoard, targetWorld, keyposes, ...
    plannedPathWorld, eeTrail, tCurrent, forceVisual, forceHud)
if ~isgraphics(ax)
    return;
end

try
cla(ax);
show(robot, q, "Parent", ax, "Frames", "off", "Visuals", "on", "PreservePlot", false);
hold(ax, "on");

draw_world_board_for_interactive(ax, cfgBoard, targetWorld, forceVisual);

if ~isempty(plannedPathWorld)
    plot3(ax, plannedPathWorld(:, 1), plannedPathWorld(:, 2), plannedPathWorld(:, 3), ...
        "Color", [0.10 0.35 0.90], "LineWidth", 2.0, "LineStyle", "-");
end

if ~isempty(keyposes)
    keyposePoints = [keyposes.hover; keyposes.contact; keyposes.hit; keyposes.back];
    plot3(ax, keyposePoints(:, 1), keyposePoints(:, 2), keyposePoints(:, 3), ...
        "m--o", "LineWidth", 1.2, "MarkerSize", 4);
end

if size(eeTrail, 1) >= 2
    plot3(ax, eeTrail(:, 1), eeTrail(:, 2), eeTrail(:, 3), ...
        "Color", [0.05 0.35 0.9], "LineWidth", 1.4);
end

format_robot_axes(ax, cfgBoard);
update_force_hud(forceHud, forceVisual);
title(ax, sprintf("Interactive impact force control, t = %.2f s, F_n = %.2f N, peak = %.2f N", ...
    tCurrent, forceVisual.force, forceVisual.peak_force));
drawnow limitrate;
catch err
    if isgraphics(ax)
        rethrow(err);
    end
end
end

function draw_world_board_for_interactive(ax, cfgBoard, targetWorld, forceVisual)
halfWidth = cfgBoard.width / 2;
halfHeight = cfgBoard.height / 2;
boardCorners = [
    -halfWidth, -halfHeight, 0;
     halfWidth, -halfHeight, 0;
     halfWidth,  halfHeight, 0;
    -halfWidth,  halfHeight, 0
];
worldCorners = (cfgBoard.R_world_board * boardCorners.').' + cfgBoard.center_world;

patch(ax, ...
    "XData", worldCorners(:, 1), ...
    "YData", worldCorners(:, 2), ...
    "ZData", worldCorners(:, 3), ...
    "FaceColor", [0.92 0.92 0.92], ...
    "FaceAlpha", 0.42, ...
    "EdgeColor", [0.2 0.2 0.2], ...
    "LineWidth", 1.2);

holePoints = cfgBoard.holes_world;
holePoints(:, 3) = cfgBoard.z_hit;
scatter3(ax, holePoints(:, 1), holePoints(:, 2), holePoints(:, 3), ...
    38, [0.15 0.15 0.15], "filled");

if ~isempty(targetWorld)
    draw_mole(ax, cfgBoard, targetWorld, forceVisual);
end
end

function draw_mole(ax, cfgBoard, targetWorld, state)
if isempty(state)
    state.drop = 0.0;
    state.force_ratio = 0.0;
    state.success = false;
end

radius = max(0.014, 0.85 * cfgBoard.hit_threshold);
holeRadius = radius * 1.35;
holeZ = cfgBoard.center_world(3) + 0.0015;
theta = linspace(0, 2 * pi, 44);

fill3(ax, ...
    targetWorld(1) + holeRadius * cos(theta), ...
    targetWorld(2) + holeRadius * sin(theta), ...
    holeZ * ones(size(theta)), ...
    [0.04 0.04 0.045], ...
    "EdgeColor", [0.01 0.01 0.01], ...
    "FaceAlpha", 0.88);

drop = min(0.034, max(0.0, state.drop));
topZ = cfgBoard.z_hit - drop;
bottomZ = cfgBoard.center_world(3) - 0.024 - 0.25 * drop;
if topZ <= bottomZ + 0.002
    topZ = bottomZ + 0.002;
end

[cx, cy, cz] = cylinder(radius, 36);
cz = bottomZ + (topZ - bottomZ) * cz;
surf(ax, ...
    targetWorld(1) + cx, ...
    targetWorld(2) + cy, ...
    cz, ...
    "FaceColor", [0.82 0.10 0.07], ...
    "EdgeColor", "none", ...
    "FaceAlpha", 0.98);

capColor = [0.98, 0.22 + 0.35 * state.force_ratio, 0.10];
fill3(ax, ...
    targetWorld(1) + radius * cos(theta), ...
    targetWorld(2) + radius * sin(theta), ...
    topZ * ones(size(theta)), ...
    capColor, ...
    "EdgeColor", [0.48 0.05 0.04], ...
    "LineWidth", 0.6);

if state.success
    scatter3(ax, targetWorld(1), targetWorld(2), cfgBoard.z_hit + 0.03, ...
        38, [0.10 0.65 0.18], "filled");
end
end

function format_robot_axes(ax, cfgBoard)
axis(ax, "equal");
grid(ax, "on");
xlim(ax, [0.0, cfgBoard.center_world(1) + cfgBoard.width / 2 + 0.20]);
ylim(ax, cfgBoard.center_world(2) + [-cfgBoard.height / 2 - 0.22, cfgBoard.height / 2 + 0.22]);
zlim(ax, [-0.05, 0.70]);
view(ax, 135, 25);
xlabel(ax, "x / m");
ylabel(ax, "y / m");
zlabel(ax, "z / m");
end

function hud = create_force_hud(fig)
figure(fig);
hud.text = annotation(fig, "textbox", [0.02 0.68 0.32 0.28], ...
    "String", "", ...
    "FitBoxToText", "off", ...
    "FontName", "Consolas", ...
    "FontSize", 9, ...
    "FontWeight", "bold", ...
    "Color", [0.04 0.05 0.06], ...
    "BackgroundColor", [1.0 1.0 1.0], ...
    "EdgeColor", [0.28 0.28 0.28], ...
    "LineWidth", 0.8, ...
    "Margin", 6);
hud.barBack = annotation(fig, "rectangle", [0.04 0.645 0.28 0.024], ...
    "FaceColor", [0.86 0.86 0.86], ...
    "Color", [0.30 0.30 0.30], ...
    "LineWidth", 0.7);
hud.barFill = annotation(fig, "rectangle", [0.04 0.645 0.001 0.024], ...
    "FaceColor", [0.85 0.18 0.12], ...
    "Color", "none");
end

function update_force_hud(hud, state)
if isempty(hud) || isempty(state) || ~state.has_force
    set(hud.text, "String", "click a target on the board");
    set(hud.barFill, "Position", [0.04 0.645 0.001 0.024], ...
        "FaceColor", [0.85 0.18 0.12]);
    return;
end

phaseText = char(state.phase);
if isempty(phaseText)
    phaseText = 'n/a';
end

threshold = max(state.threshold, eps);
fillRatio = min(1.0, max(0.0, state.force / threshold));
barColor = [0.85 0.18 0.12];
if state.force >= state.threshold
    barColor = [0.12 0.62 0.22];
end
hudText = sprintf([ ...
    'contact force  %5.2f N\n', ...
    'filtered       %5.2f N\n', ...
    'peak           %5.2f N\n', ...
    'threshold      %5.2f N\n', ...
    'target         %5.2f N\n', ...
    'phase          %s'], ...
    state.force, state.filtered_force, state.peak_force, ...
    state.threshold, state.target_force, phaseText);

set(hud.text, "String", hudText);
set(hud.barFill, "Position", [0.04 0.645 0.28 * fillRatio 0.024], ...
    "FaceColor", barColor);
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

function y = smoothstep(x)
x = min(max(x, 0.0), 1.0);
y = x * x * (3 - 2 * x);
end
