requestedHitMotionMode = "";
if exist("hitMotionMode", "var")
    requestedHitMotionMode = hitMotionMode;
elseif exist("motionMode", "var")
    requestedHitMotionMode = motionMode;
end
clearvars -except requestedHitMotionMode; clc; close all;

% 交互式 RRT 避障打地鼠主仿真脚本。
% 运行方式：
%   run("matlab/main_interactive_sim.m")
%
% 使用方法：
%   1. 在 2D 目标板窗口中点击任意安全区域。
%   2. 合法点击会触发在线 RRT 避障规划，并抢占当前正在执行的轨迹。
%   3. 点击障碍物内部会被立即拦截，不会打断机械臂当前动作。
%   4. 关闭任一窗口，或在目标板窗口按 Esc，停止仿真循环。

rootDir = fileparts(fileparts(mfilename("fullpath")));
addpath(genpath(fullfile(rootDir, "matlab")));

cfgRobot = config_robot();
cfgBoard = config_board();
cfgController = config_controller();
cfgController = apply_motion_mode_override(cfgController, requestedHitMotionMode);

% 本脚本使用题目要求的 0.01 s 步进。保留配置结构体，方便所有子函数继续复用。
dt = 0.01;
cfgController.dt = dt;
cfgController.path_type = "rrt";

robot = build_robot(cfgRobot);

% -------------------------------------------------------------------------
% 1. 2D 障碍物配置。
%    坐标全部位于目标板坐标系 {G} 下，单位 m。
%    这里先使用圆形障碍；collision_check_2d/draw_obstacles 也支持矩形。
% -------------------------------------------------------------------------
obstacles = make_default_obstacles();
rrtBounds = board_bounds(cfgBoard);
rrtOptions.step_size = 0.025;
rrtOptions.goal_threshold = 0.025;
rrtOptions.max_iter = 2500;
rrtOptions.goal_bias = 0.16;
rrtOptions.collision_resolution = 0.004;

numJoints = numel(cfgRobot.home_configuration);
q_act = cfgRobot.home_configuration(:).';
qd_act = zeros(1, numJoints);

% 当前正在执行的轨迹及其本地计时器。没有目标时，机械臂保持 hold_q。
activeTraj = [];
t_traj = 0;
hold_q = q_act;

currentTargetWorld = [];
currentKeyposes = [];
currentRrtPathBoard = [];
currentRrtPathWorld = [];
eeTrail = zeros(0, 3);

t_current = 0;
stepIndex = 0;
graphicsEvery = 5;  % 100 Hz 仿真步进，约 20 Hz 图形刷新，避免 show() 太慢。

[boardFig, boardAx, ~, eeMarker2d, pathLine2d] = create_board_window(cfgBoard, obstacles);
[robotFig, robotAx] = create_robot_window();

show_robot_frame(robotAx, robot, q_act, cfgRobot, cfgBoard, obstacles, ...
    currentTargetWorld, currentKeyposes, currentRrtPathWorld, eeTrail, t_current);

fprintf("Interactive RRT simulation started.\n");
fprintf("Hit motion mode: %s\n", cfgController.motion_mode);
fprintf("Click a safe point on the 2D board. Clicks inside obstacles will be rejected.\n");
fprintf("Press Esc in the board window, or close a figure, to stop.\n");

while is_simulation_running(boardFig, robotFig)
    stepTimer = tic;

    % ---------------------------------------------------------------------
    % 2. 轨迹抢占检测。
    %    鼠标回调只负责合法性拦截和写 appdata；真正的 RRT/IK/轨迹规划
    %    放在主循环里执行，避免 GUI 回调阻塞或和动力学状态读写打架。
    % ---------------------------------------------------------------------
    if getappdata(boardFig, "is_new_target")
        setappdata(boardFig, "is_new_target", false);

        targetBoard = getappdata(boardFig, "target_board");
        targetWorld = getappdata(boardFig, "target_world");

        % 抢占瞬间截取实际关节状态。新轨迹必须从这两个边界条件出发。
        q_start = q_act;
        qd_start = qd_act;

        % -----------------------------------------------------------------
        % 关键点：动态提取 RRT 起点。
        % 用 FK 得到当前末端在世界坐标系的位置，再投影回目标板 2D 坐标。
        % 注意：RRT 起点不是上一次目标点，也不是轨迹参考点，而是当前实际机械臂末端位置。
        % 这样用户连续点击时，新规划真正从“打断瞬间”开始。
        % -----------------------------------------------------------------
        eeWorld = get_end_effector_position(robot, q_start, cfgRobot);
        rrtStartBoard = world_to_board(eeWorld, cfgBoard);
        rrtGoalBoard = targetBoard;

        try
            [newTraj, planInfo] = plan_rrt_preempted_hit_traj( ...
                robot, rrtStartBoard, rrtGoalBoard, obstacles, rrtBounds, rrtOptions, ...
                cfgRobot, cfgBoard, cfgController, q_start, qd_start);

            activeTraj = newTraj;
            currentTargetWorld = targetWorld;
            currentKeyposes = planInfo.keyposes;
            currentRrtPathBoard = planInfo.path_board;
            currentRrtPathWorld = planInfo.path_world_hover;
            hold_q = activeTraj.q_des(end, :);
            t_traj = 0;

            update_rrt_path_line(pathLine2d, currentRrtPathBoard);
            title(boardAx, sprintf("RRT 规划成功：%d 个平滑路点，开始抢占执行", ...
                size(currentRrtPathBoard, 1)), "Color", [0.0 0.45 0.15]);

            fprintf("t=%.2f s: accepted target board=[%.3f %.3f], motion=%s, RRT nodes=%d, smooth waypoints=%d\n", ...
                t_current, targetBoard(1), targetBoard(2), ...
                cfgController.motion_mode, ...
                size(planInfo.raw_nodes, 1), size(currentRrtPathBoard, 1));
        catch err
            % RRT 或 IK 失败时，不覆盖 activeTraj。机械臂继续执行上一次有效轨迹。
            warning("main_interactive_sim:RRTReplanFailed", ...
                "RRT replan failed. Keeping previous trajectory. Detail: %s", err.message);
            title(boardAx, "RRT 规划失败：保留上一条有效轨迹，请重新选择", ...
                "Color", [0.80 0.05 0.05]);
        end
    end

    % ---------------------------------------------------------------------
    % 3. 当前参考轨迹采样。
    %    如果没有活动轨迹，则保持上一次完成姿态或 home 姿态。
    % ---------------------------------------------------------------------
    if isempty(activeTraj)
        q_ref = hold_q;
        qd_ref = zeros(1, numJoints);
        qdd_ref = zeros(1, numJoints);
    else
        [q_ref, qd_ref, qdd_ref, isTrajDone] = sample_joint_traj(activeTraj, t_traj);
        if isTrajDone
            hold_q = activeTraj.q_des(end, :);
            activeTraj = [];
            t_traj = 0;
        else
            t_traj = t_traj + dt;
        end
    end

    % ---------------------------------------------------------------------
    % 4. 控制器与简化动力学步进。
    %    这里调用现有 gravity_pd_controller() 得到力矩，再用单位惯量近似积分。
    %    后续若要换成更真实的 rigidBodyTree 动力学，只需替换 step_joint_dynamics()。
    % ---------------------------------------------------------------------
    tau = gravity_pd_controller(robot, q_act, qd_act, q_ref, qd_ref, cfgController);
    [q_act, qd_act, ~] = step_joint_dynamics(robot, q_act, qd_act, qdd_ref, tau, dt);

    % ---------------------------------------------------------------------
    % 5. 实时可视化更新：2D 板上末端投影 + 3D 机器人、障碍物和 RRT 路径。
    % ---------------------------------------------------------------------
    if mod(stepIndex, graphicsEvery) == 0
        eePosition = get_end_effector_position(robot, q_act, cfgRobot);
        eeTrail = [eeTrail; eePosition]; %#ok<AGROW>
        if size(eeTrail, 1) > 600
            eeTrail = eeTrail(end-599:end, :);
        end

        update_board_ee_marker(eeMarker2d, eePosition, cfgBoard);
        show_robot_frame(robotAx, robot, q_act, cfgRobot, cfgBoard, obstacles, ...
            currentTargetWorld, currentKeyposes, currentRrtPathWorld, eeTrail, t_current);
    end

    t_current = t_current + dt;
    stepIndex = stepIndex + 1;

    pause(max(0, dt - toc(stepTimer)));
end

fprintf("Interactive RRT simulation stopped at t=%.2f s.\n", t_current);

function obstacles = make_default_obstacles()
% 定义目标板 2D 坐标系下的障碍物。
% type="circle" 时需要 center 和 radius 字段。
obstacles = [
    struct("type", "circle", "center", [-0.11,  0.055], "radius", 0.045)
    struct("type", "circle", "center", [ 0.08, -0.060], "radius", 0.052)
    struct("type", "circle", "center", [ 0.18,  0.070], "radius", 0.035)
];
end

function bounds = board_bounds(cfgBoard)
% RRT 采样边界：[xmin xmax ymin ymax]，限制在目标板矩形范围内。
bounds = [-cfgBoard.width / 2, cfgBoard.width / 2, ...
          -cfgBoard.height / 2, cfgBoard.height / 2];
end

function [fig, ax, targetMarker, eeMarker, pathLine] = create_board_window(cfgBoard, obstacles)
% 创建 2D 目标板窗口，并绑定点击拦截回调。
fig = figure("Name", "Interactive RRT Target Board", "Color", "w", "NumberTitle", "off");
ax = axes("Parent", fig);
axes(ax);

draw_board(cfgBoard, []);
draw_obstacles(obstacles);
title(ax, "点击安全区域设置目标；障碍物内部点击会被拒绝；Esc 停止", ...
    "Color", [0.1 0.1 0.1]);

hold(ax, "on");
pathLine = plot(ax, NaN, NaN, "Color", [0.10 0.35 0.90], ...
    "LineWidth", 2.0, "LineStyle", "-");
targetMarker = plot(ax, NaN, NaN, "ro", "MarkerSize", 16, "LineWidth", 2.5);
eeMarker = plot(ax, NaN, NaN, "bo", "MarkerSize", 9, "LineWidth", 1.5);
legend(ax, [pathLine, targetMarker, eeMarker], ...
    ["RRT path", "target", "end effector"], "Location", "southoutside");

setappdata(fig, "target_board", []);
setappdata(fig, "target_world", []);
setappdata(fig, "is_new_target", false);
setappdata(fig, "is_running", true);

set(fig, "WindowButtonDownFcn", ...
    @(src, event)on_board_click(src, event, cfgBoard, obstacles, ax, targetMarker, pathLine));
set(fig, "KeyPressFcn", @(src, event)on_key_press(src, event));
end

function [fig, ax] = create_robot_window()
% 创建 3D 机器人窗口。
fig = figure("Name", "Interactive UR5 RRT Hit Simulation", "Color", "w", "NumberTitle", "off");
ax = axes("Parent", fig);
end

function on_board_click(fig, ~, cfgBoard, obstacles, ax, targetMarker, pathLine)
% 鼠标点击回调。
% 这里实现“目标点合法性校验/拦截机制”：非法点会立刻被拒绝，
% 不写入新目标，不触发 is_new_target，机械臂继续执行上一条有效轨迹。
if ~strcmp(get(fig, "SelectionType"), "normal")
    return;
end

point = get(ax, "CurrentPoint");
clickedPoint = point(1, 1:2);

if ~is_inside_board(clickedPoint, cfgBoard)
    return;
end

% -------------------------------------------------------------------------
% 关键安全逻辑：点碰撞拦截。
% collision_check_2d 已扩展为双模式：
%   collision_check_2d(point, obstacles)             -> 点是否在障碍物内
%   collision_check_2d(p1, p2, obstacles, resolution)-> 线段是否碰撞
% 点击回调必须先做点检测，只有安全点才允许进入主循环重规划。
% -------------------------------------------------------------------------
if collision_check_2d(clickedPoint, obstacles)
    setappdata(fig, "is_new_target", false);
    warning("main_interactive_sim:TargetInsideObstacle", ...
        "目标点在障碍物内，已拒绝该输入！");
    title(ax, "非法目标：处于障碍物内部！请重新选择。", ...
        "Color", [0.85 0.05 0.05]);
    drawnow limitrate;
    return;
end

targetWorld = board_to_world(clickedPoint, cfgBoard);

setappdata(fig, "target_board", clickedPoint);
setappdata(fig, "target_world", targetWorld);
setappdata(fig, "is_new_target", true);

if isgraphics(targetMarker)
    set(targetMarker, "XData", clickedPoint(1), "YData", clickedPoint(2));
end
if isgraphics(pathLine)
    set(pathLine, "XData", NaN, "YData", NaN);
end
title(ax, "目标设定成功，正在重规划避障路径...", ...
    "Color", [0.0 0.48 0.18]);
drawnow limitrate;
end

function on_key_press(fig, event)
% 按 Esc 停止主循环；关闭任一窗口也会停止。
if strcmp(event.Key, "escape")
    setappdata(fig, "is_running", false);
end
end

function inside = is_inside_board(pointBoard, cfgBoard)
% 点击必须在目标板矩形范围内。
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

function [traj, planInfo] = plan_rrt_preempted_hit_traj( ...
    robot, rrtStartBoard, rrtGoalBoard, obstacles, rrtBounds, rrtOptions, ...
    cfgRobot, cfgBoard, cfgController, qStart, qdStart)
% 在线 RRT 避障 + 多段关节轨迹生成。
%
% 输入：
%   rrtStartBoard : 由当前实际末端 FK 投影得到的 2D 起点
%   rrtGoalBoard  : 用户最新点击的合法 2D 目标点
%   qStart/qdStart: 抢占瞬间的实际关节状态
%
% 输出：
%   traj           : 可被步进循环采样执行的关节轨迹
%   planInfo       : 用于可视化/调试的 RRT 路径和关键姿态

if collision_check_2d(rrtGoalBoard, obstacles)
    error("The RRT goal is inside an obstacle. This should have been intercepted by the click callback.");
end

rawStartBoard = rrtStartBoard;
if collision_check_2d(rrtStartBoard, obstacles)
    % 理论上，上一条有效 RRT 轨迹会让末端投影避开障碍物。
    % 但初始 home 姿态或简化动力学误差可能让 FK 投影恰好落入障碍。
    % 此时不能改变真实 qStart；这里只微调“RRT 采样树的二维起点”，
    % 让规划器能从最近安全点扩展，后续关节轨迹仍从 qStart/qdStart 平滑接上。
    rrtStartBoard = nearest_safe_planner_start(rrtStartBoard, rrtGoalBoard, obstacles, rrtBounds);
    warning("main_interactive_sim:RRTStartInsideObstacle", ...
        "RRT start projection [%.3f %.3f] is inside an obstacle; planner start adjusted to [%.3f %.3f].", ...
        rawStartBoard(1), rawStartBoard(2), rrtStartBoard(1), rrtStartBoard(2));
end

if norm(rrtStartBoard - rrtGoalBoard) < rrtOptions.goal_threshold
    rawPath = [rrtStartBoard; rrtGoalBoard];
    info.found = true;
    info.iterations = 0;
    info.nodes = rawPath;
    info.parents = [0; 1];
else
    [rawPath, info] = rrt_plan_2d(rrtStartBoard, rrtGoalBoard, obstacles, rrtBounds, rrtOptions);
end

if ~info.found || isempty(rawPath)
    error("RRT did not find a collision-free path after %d iterations.", info.iterations);
end

smoothPath = smooth_path(rawPath, obstacles, 120);
smoothPath = ensure_path_endpoints(smoothPath, rrtStartBoard, rrtGoalBoard);
assert_collision_free_path(smoothPath, obstacles, rrtOptions.collision_resolution);

% -------------------------------------------------------------------------
% 关键逻辑：把 2D RRT 路径转换成机械臂关节空间路点。
% 1. RRT 在板坐标系中只负责 XY 避障。
% 2. 每个 RRT 点先经 board_to_world() 转为世界坐标。
% 3. 避障移动阶段统一使用 z_hover，高于板面，避免沿板面“刮过去”。
% 4. 第一个关节路点不重新 IK，而是直接使用 qStart，保证轨迹从真实状态接上。
% 5. RRT 到达目标上方后，再追加 Hit 和 Back 两个动作完成敲击。
% -------------------------------------------------------------------------
[qWaypoints, pathWorldHover, keyposes] = rrt_path_to_joint_waypoints( ...
    robot, smoothPath, rrtGoalBoard, cfgRobot, cfgBoard, qStart);

% -------------------------------------------------------------------------
% 关键逻辑：速度边界对齐。
% qdWaypoints 第一行使用抢占瞬间实际速度 qdStart，避免新轨迹一开始
% 速度突变；最后一行强制为 0，保证打击返回后稳定停住。
% hover_stop 模式只在目标上方 hover waypoint 置零；continuous 模式
% 则让轨迹连续穿过该点直接下捶。
% -------------------------------------------------------------------------
hoverWaypointIndex = size(qWaypoints, 1) - 2;
[qdWaypoints, qddWaypoints] = make_hit_motion_boundaries( ...
    qWaypoints, cfgController, qdStart, hoverWaypointIndex);

traj = plan_joint_traj(qWaypoints, cfgController.segment_time, cfgController.dt, ...
    qdWaypoints, qddWaypoints);

traj.path_type = "rrt";
traj.motion_mode = cfgController.motion_mode;
traj.rrt_path_board = smoothPath;
traj.rrt_path_world_hover = pathWorldHover;

planInfo.raw_path = rawPath;
planInfo.raw_nodes = info.nodes;
planInfo.raw_start_board = rawStartBoard;
planInfo.rrt_start_board = rrtStartBoard;
planInfo.path_board = smoothPath;
planInfo.path_world_hover = pathWorldHover;
planInfo.keyposes = keyposes;
planInfo.rrt_info = info;
end

function safeStart = nearest_safe_planner_start(startPoint, goalPoint, obstacles, bounds)
% 当 FK 投影起点意外落入障碍物时，为 RRT 找一个最近的安全二维起点。
% 优先沿“当前点 -> 目标点”方向向外搜索；如果整条射线都不可用，再做小半径环形搜索。
alphaList = linspace(0, 1, 50).';
for i = 2:numel(alphaList)
    candidate = startPoint + alphaList(i) * (goalPoint - startPoint);
    if is_point_in_bounds(candidate, bounds) && ~collision_check_2d(candidate, obstacles)
        safeStart = candidate;
        return;
    end
end

radii = 0.005:0.005:0.12;
theta = linspace(0, 2*pi, 64);
for radius = radii
    for angle = theta
        candidate = startPoint + radius * [cos(angle), sin(angle)];
        if is_point_in_bounds(candidate, bounds) && ~collision_check_2d(candidate, obstacles)
            safeStart = candidate;
            return;
        end
    end
end

error("Cannot find a nearby collision-free planner start around the current end-effector projection.");
end

function inside = is_point_in_bounds(point, bounds)
inside = point(1) >= bounds(1) && point(1) <= bounds(2) && ...
    point(2) >= bounds(3) && point(2) <= bounds(4);
end

function path = ensure_path_endpoints(path, startPoint, goalPoint)
% smooth_path 正常会保留首尾点；这里再显式兜底，方便后续可视化和 IK。
if isempty(path)
    path = [startPoint; goalPoint];
    return;
end
if norm(path(1, :) - startPoint) > 1e-9
    path = [startPoint; path];
end
if norm(path(end, :) - goalPoint) > 1e-9
    path = [path; goalPoint];
end
end

function assert_collision_free_path(path, obstacles, resolution)
% 对平滑后的路径逐段复查，防止剪枝后意外穿过障碍物。
for i = 1:(size(path, 1) - 1)
    if collision_check_2d(path(i, :), path(i + 1, :), obstacles, resolution)
        error("Smoothed RRT path segment %d collides with an obstacle.", i);
    end
end
end

function [qWaypoints, pathWorldHover, keyposes] = rrt_path_to_joint_waypoints( ...
    robot, pathBoard, goalBoard, cfgRobot, cfgBoard, qStart)
% 将 2D RRT 路径转换成关节空间多段路点。
numPathPoints = size(pathBoard, 1);
pathWorldHover = zeros(numPathPoints, 3);
for i = 1:numPathPoints
    pointWorld = board_to_world(pathBoard(i, :), cfgBoard);
    pointWorld(3) = cfgBoard.z_hover;
    pathWorldHover(i, :) = pointWorld;
end

targetWorld = board_to_world(goalBoard, cfgBoard);
keyposes = make_hit_keyposes(targetWorld, cfgBoard);

qWaypoints = qStart;
qSeed = qStart;

% 跳过第一个 RRT 点：第一个点只是当前末端投影，实际起始关节必须是 qStart。
for i = 2:numPathPoints
    qNext = solve_ik(robot, pathWorldHover(i, :), cfgRobot, qSeed);
    qWaypoints = [qWaypoints; qNext]; %#ok<AGROW>
    qSeed = qNext;
end

% 目标上方已经由 RRT 路径最后一个点覆盖。随后追加真正的敲击和返回。
qHit = solve_ik(robot, keyposes.hit, cfgRobot, qSeed);
qBack = solve_ik(robot, keyposes.back, cfgRobot, qHit);
qWaypoints = [qWaypoints; qHit; qBack];
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

function [qNext, qdNext, qddAct] = step_joint_dynamics(robot, qAct, qdAct, qddRef, tau, dt)
% 简化关节动力学步进。
% 当前没有完整质量矩阵仿真，因此采用单位惯量近似：
%   qdd ~= qdd_ref + tau_servo
% tau_servo 是从 gravity_pd_controller 输出中扣除重力项后的伺服力矩。
tauServo = tau(:).';
try
    tauGravity = gravityTorque(robot, qAct);
    tauServo = tauServo - tauGravity(:).';
catch
    % 如果重力项不可用，tauServo 已经等价于普通 PD 输出。
end

qddAct = qddRef(:).' + tauServo;
maxAbsAcceleration = 45;
qddAct = min(max(qddAct, -maxAbsAcceleration), maxAbsAcceleration);

qdNext = qdAct + qddAct * dt;
qNext = qAct + qdNext * dt;
end

function eePosition = get_end_effector_position(robot, q, cfgRobot)
pose = getTransform(robot, q, cfgRobot.end_effector);
eePosition = tform2trvec(pose);
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

function update_rrt_path_line(pathLine, pathBoard)
if ~isgraphics(pathLine) || isempty(pathBoard)
    return;
end
set(pathLine, "XData", pathBoard(:, 1), "YData", pathBoard(:, 2));
end

function show_robot_frame(ax, robot, q, cfgRobot, cfgBoard, obstacles, ...
    targetWorld, keyposes, rrtPathWorld, eeTrail, tCurrent)
if ~isgraphics(ax)
    return;
end

cla(ax);
show(robot, q, "Parent", ax, "Frames", "off", "Visuals", "on", "PreservePlot", false);
hold(ax, "on");

draw_world_board_for_interactive(ax, cfgBoard, obstacles, targetWorld);

if ~isempty(rrtPathWorld)
    plot3(ax, rrtPathWorld(:, 1), rrtPathWorld(:, 2), rrtPathWorld(:, 3), ...
        "Color", [0.10 0.35 0.90], "LineWidth", 2.0, "LineStyle", "-");
end

if ~isempty(keyposes)
    keyposePoints = [keyposes.hover; keyposes.hit; keyposes.back];
    plot3(ax, keyposePoints(:, 1), keyposePoints(:, 2), keyposePoints(:, 3), ...
        "m--o", "LineWidth", 1.2, "MarkerSize", 4);
end

if size(eeTrail, 1) >= 2
    plot3(ax, eeTrail(:, 1), eeTrail(:, 2), eeTrail(:, 3), ...
        "Color", [0.05 0.35 0.9], "LineWidth", 1.4);
end

axis(ax, "equal");
grid(ax, "on");
xlim(ax, [0.0, cfgBoard.center_world(1) + cfgBoard.width / 2 + 0.20]);
ylim(ax, cfgBoard.center_world(2) + [-cfgBoard.height / 2 - 0.22, cfgBoard.height / 2 + 0.22]);
zlim(ax, [-0.05, 0.70]);
view(ax, 135, 25);
xlabel(ax, "x / m");
ylabel(ax, "y / m");
zlabel(ax, "z / m");
title(ax, sprintf("Interactive UR5 RRT simulation, t = %.2f s, ee = %s", ...
    tCurrent, cfgRobot.end_effector));
drawnow limitrate;
end

function draw_world_board_for_interactive(ax, cfgBoard, obstacles, targetWorld)
% 在 3D 世界坐标中绘制目标板、孔位、障碍物和当前目标。
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

draw_world_obstacles(ax, obstacles, cfgBoard);

if ~isempty(targetWorld)
    targetPoint = [targetWorld(1), targetWorld(2), cfgBoard.z_hit];
    scatter3(ax, targetPoint(1), targetPoint(2), targetPoint(3), ...
        120, [0.9 0.05 0.05], "filled");
end
end

function draw_world_obstacles(ax, obstacles, cfgBoard)
% 将 2D 障碍物投影到 3D 目标板上显示，帮助对照 RRT 路径。
for k = 1:numel(obstacles)
    obstacle = obstacles(k);
    switch obstacle.type
        case "circle"
            theta = linspace(0, 2*pi, 80).';
            boardXY = obstacle.center + obstacle.radius * [cos(theta), sin(theta)];
            worldXYZ = zeros(size(boardXY, 1), 3);
            for i = 1:size(boardXY, 1)
                worldXYZ(i, :) = board_to_world(boardXY(i, :), cfgBoard);
            end
            worldXYZ(:, 3) = cfgBoard.z_hit + 0.003;
            patch(ax, ...
                "XData", worldXYZ(:, 1), ...
                "YData", worldXYZ(:, 2), ...
                "ZData", worldXYZ(:, 3), ...
                "FaceColor", [0.85 0.18 0.10], ...
                "FaceAlpha", 0.30, ...
                "EdgeColor", [0.75 0.10 0.06], ...
                "LineWidth", 1.0);
        case "rect"
            b = obstacle.bounds;
            boardXY = [
                b(1), b(3);
                b(2), b(3);
                b(2), b(4);
                b(1), b(4)
            ];
            worldXYZ = zeros(size(boardXY, 1), 3);
            for i = 1:size(boardXY, 1)
                worldXYZ(i, :) = board_to_world(boardXY(i, :), cfgBoard);
            end
            worldXYZ(:, 3) = cfgBoard.z_hit + 0.003;
            patch(ax, ...
                "XData", worldXYZ(:, 1), ...
                "YData", worldXYZ(:, 2), ...
                "ZData", worldXYZ(:, 3), ...
                "FaceColor", [0.85 0.18 0.10], ...
                "FaceAlpha", 0.30, ...
                "EdgeColor", [0.75 0.10 0.06], ...
                "LineWidth", 1.0);
    end
end
end
