# 目标点到敲击动作的规划与控制流程

本文说明项目中的一条基本闭环：机械臂收到一个目标点后，先运动到目标点上方，再沿 z 方向下压完成敲击，最后回撤。力控细节已经在 [impact_force_control.md](impact_force_control.md) 中展开，本文只保留必要接口和整体位置。

## 1. 总体链路

整套流程可以拆成 8 个阶段：

```text
目标输入
  -> 目标点归一化
  -> 板坐标到世界坐标
  -> 构造 hover/contact/press/back 关键点
  -> 逆运动学求关节路点
  -> 五次多项式生成关节轨迹
  -> 控制器跟踪轨迹并处理接触
  -> 根据位置、接触力和持续时间判断敲击结果
```

核心文件关系如下：

```text
目标输入:
  matlab/target/generate_random_target.m
  matlab/target/board_target_from_id.m
  matlab/communication/read_latest_udp_board_target.m

坐标与目标板:
  matlab/config/config_board.m
  matlab/target/board_to_world.m

关键点与轨迹:
  matlab/planning/make_hit_keyposes.m
  matlab/planning/make_hit_motion_boundaries.m
  matlab/planning/plan_joint_traj.m
  matlab/robot/solve_ik.m

基线位置控制:
  matlab/target/execute_hit_target.m
  matlab/control/simulate_joint_tracking.m
  matlab/control/simulate_contact_force.m

高速度敲击与力控:
  matlab/main_impact_force_control.m
  matlab/main_interactive_impact_force_control.m
  matlab/main_udp_impact_force_control.m
  matlab/control/simulate_impact_force_control.m
```

## 2. 目标点输入

项目中目标点有三种来源，但进入规划层后都会变成同一种数据：`targetWorld = [x, y, z]`。

- 随机目标：`generate_random_target(cfgBoard)` 从九宫格孔位中随机选择一个目标。
- 鼠标目标：交互脚本读取目标板窗口中的点击坐标，检查是否在板内，再转成世界坐标。
- UDP 目标：`read_latest_udp_board_target()` 接收 Python 端发来的 JSON，支持 `target_id`、`id`、`row/col`、`board` 和 `world` 等字段。

UDP 接收逻辑会清空积压包，只保留最新合法目标。这样视觉或 Python 端连续发目标时，MATLAB 不会按旧包排队执行，而是尽量响应最新目标。

目标板配置在 `config_board.m`：

```matlab
cfg.center_world = [0.45, 0.0, 0.05];
cfg.x_list = [-0.20, 0, 0.20];
cfg.y_list = [-0.13, 0, 0.13];
cfg.z_hover = cfg.center_world(3) + 0.10;
cfg.z_hit = cfg.center_world(3) + 0.01;
cfg.hit_threshold = 0.015;
```

其中 `z_hover` 是目标点上方安全高度，`z_hit` 是地鼠顶部或敲击接触高度，`hit_threshold` 用于判断末端 XY 是否打在目标孔附近。

## 3. 坐标映射

目标检测或点击通常先得到板坐标 `{G}` 下的二维点：

```text
pointBoard = [x_board, y_board]
```

`board_to_world()` 将它转换到世界坐标：

```matlab
pointWorld = cfgBoard.R_world_board * pointBoard3 + cfgBoard.center_world;
```

当前配置中 `R_world_board = eye(3)`，所以目标板没有相对世界坐标旋转。后续如果相机标定或目标板姿态发生变化，只需要修改 `center_world` 和 `R_world_board`，规划链路本身不需要改。

九宫格编号由 `board_target_from_id()` 管理，按行优先排列：

```text
1 2 3   y = -0.13
4 5 6   y =  0.00
7 8 9   y =  0.13
```

## 4. 敲击关键点

直接敲击不先做复杂三维路径搜索，而是围绕目标点生成固定语义的关键点。`make_hit_keyposes()` 的逻辑是：

```matlab
keyposes.hover   = [targetWorld(1), targetWorld(2), cfgBoard.z_hover];
keyposes.contact = [targetWorld(1), targetWorld(2), cfgBoard.z_hit];
keyposes.press   = [targetWorld(1), targetWorld(2), cfgBoard.z_hit - cfgForce.press_depth];
keyposes.hit     = keyposes.press;
keyposes.back    = keyposes.hover;
```

含义如下：

- `hover`：目标点正上方，用于快速移动到敲击准备位置。
- `contact`：接触高度，对应地鼠顶面附近。
- `press/hit`：向下压入深度 `press_depth`，用于产生足够接触力。
- `back`：回到 hover 高度，完成一次敲击后的安全回撤。

这几个点把任务空间动作写成很清楚的“上方准备、接触、下压、回撤”。后续所有 IK 和关节轨迹生成都基于这组点。

## 5. 逆运动学

`solve_ik()` 把每个任务空间关键点转成 UR5 关节角：

```matlab
targetPose = trvec2tform(pointWorld) * eul2tform(cfgRobot.fixed_tool_eul_zyx, "ZYX");
q = ik(cfgRobot.end_effector, targetPose, cfgRobot.ik_weights, qSeed);
```

这里有两个重要设计：

1. 末端姿态固定  
   敲击任务主要关心末端位置和下压方向，因此工具姿态由 `cfgRobot.fixed_tool_eul_zyx` 固定，避免敲击过程中腕部姿态任意漂移。

2. IK 种子连续传递  
   后一个点的 IK 使用前一个点的解作为 `qSeed`。例如：

   ```matlab
   qHover = solve_ik(robot, keyposes.hover, cfgRobot, qStart);
   qContact = solve_ik(robot, keyposes.contact, cfgRobot, qHover);
   qHit = solve_ik(robot, keyposes.hit, cfgRobot, qContact);
   qBack = solve_ik(robot, keyposes.back, cfgRobot, qHit);
   ```

   这样可以减少 IK 跳到另一组等价解的概率，使关节轨迹更连续。

## 6. 关节轨迹规划

得到关节路点后，系统使用五次多项式连接相邻路点。基线流程中的路点顺序是：

```matlab
qWaypoints = [q0; qHover; qContact; qPress; qBack];
```

交互和 UDP 力控流程会从当前实际状态抢占重规划：

```matlab
qWaypoints = [qStart; qHover; qContact; qHit; qBack];
```

其中 `qStart/qdStart` 是收到新目标那一刻的实际关节位置和速度。这个设计很关键：连续目标不会等待上一轮动作播完，而是从当前真实机械臂状态重新规划，避免参考轨迹和实际状态断开。

`plan_joint_traj()` 对每一段求五次多项式：

```text
q(t) = a0 + a1 t + a2 t^2 + a3 t^3 + a4 t^4 + a5 t^5
```

它同时满足段首和段尾的：

```text
q, qd, qdd
```

因此关节位置、速度和加速度在路点处可控，适合用作控制器的连续参考。

## 7. 路点速度边界

`make_hit_motion_boundaries()` 负责给每个路点设置速度边界。

默认 `motion_mode = "hover_stop"` 时，机械臂会在目标上方 `hover` 点速度置零，然后再向下敲击：

```text
home/current -> hover(停一下) -> contact -> press/hit -> back
```

如果设置为 `motion_mode = "continuous"`，hover 点使用中心差分速度，轨迹会更连贯：

```text
home/current -> hover(不停) -> contact -> press/hit -> back
```

二者的取舍是：

- `hover_stop` 更稳，适合演示“先到上方再敲”的基本链路。
- `continuous` 响应更快，适合连续目标或 UDP 实时输入。

## 8. 控制执行：基线位置控制

最简单的闭环在 `execute_hit_target()` 中：

```text
关键点 -> IK -> 五次多项式关节轨迹 -> 关节 PD 跟踪 -> 事后估计接触力
```

`simulate_joint_tracking()` 使用简化关节空间 PD：

```matlab
qdd = Kp .* (qDes - q) + Kd .* (qdDes - qd);
qd = qd + qdd * dt;
q = q + qd * dt;
```

这个版本的优点是流程清楚、易于调试。它能验证“目标点输入、关键点生成、IK、轨迹跟踪、敲击判定”这条主线是否打通。

它的限制也很明确：`simulate_contact_force()` 是根据末端实际轨迹事后估算接触力，接触力没有反过来影响机械臂动力学。因此它适合做基础链路和位置规划验证，不适合说明高速碰撞瞬间的真实力学过程。

## 9. 控制执行：高速度敲击与力控

力控版本在基线链路上增加了真实一些的接触闭环：

```text
目标点 -> hover/contact/hit/back 参考轨迹
      -> 操作空间阻抗控制
      -> 接触力模型
      -> J'F_contact 外力矩
      -> 接触后导纳式 z 方向修正
      -> 成功后回撤
```

当前实现中有两种形态：

- `simulate_impact_force_control()`：离线仿真版本，直接从目标 hover 姿态开始，内部生成 `approach -> force -> retract` 状态机。
- `main_interactive_impact_force_control.m` 和 `main_udp_impact_force_control.m`：实时交互版本，先按 `qStart -> qHover -> qContact -> qHit -> qBack` 生成参考轨迹，再在逐步控制中叠加接触力状态机。

操作空间阻抗控制的核心形式是：

```text
F_task = Kx (x_des - x) + Dx (xd_des - xd)
tau_task = Jv' F_task
```

同时加入姿态保持、重力补偿和力矩限幅：

```text
tau_cmd = tau_task + tau_posture + tau_gravity
```

当末端进入目标 XY 范围并向下压入 `z_hit` 以下时，接触模型产生法向力。该力通过雅可比转置进入关节动力学：

```text
M(q) qdd + C(q,qd) + G(q) = tau_cmd + J(q)' F_contact
```

接触后，控制器不再只盲目跟踪下压轨迹，而是根据过滤后的接触力修正 z 方向命令：

```text
z_cmd = z_cmd - K_adm * (F_des - F_filtered) * dt
```

这部分只在本文中简述。完整的接触模型、传感器延迟、滤波、导纳参数和实验分析见 [impact_force_control.md](impact_force_control.md)。

## 10. 敲击成功判定

敲击不是只看末端是否到了目标点，而是同时检查位置和力。

基线位置控制中：

```text
xy_error <= cfgBoard.hit_threshold
penetration = max(0, cfgBoard.z_hit - ee_z)
normalForce >= cfgForce.threshold
above_threshold_duration >= cfgForce.min_contact_time
```

力控版本中也使用类似判据，只是接触力在仿真步内实时计算并参与动力学。关键参数在：

```text
matlab/config/config_board.m
matlab/config/config_force.m
matlab/config/config_impact_force_control.m
```

常用判据参数：

```matlab
cfgBoard.hit_threshold = 0.015;
cfgForce.threshold = 10.0;
cfgForce.min_contact_time = 0.02;
```

也就是说，末端需要在目标孔附近产生超过阈值的接触力，并维持足够时间，才认为地鼠被击中。

## 11. 可选：RRT 避障路径

`main_interactive_sim.m` 中还有一个带 RRT 的交互版本。它不是这条基本链路的必要部分，但可以看作 hover 前的扩展规划层。

RRT 只在目标板二维平面上规划 XY 路径：

```text
当前末端投影 -> 目标点投影
```

规划成功后，路径点统一抬到 `z_hover` 高度，再逐点 IK 成关节路点。到达目标上方后，仍然追加 `hit/back` 动作：

```text
RRT hover path -> hit -> back
```

因此 RRT 版本和直接版本共享同一套“目标点上方、向下敲、回撤”的尾部动作，只是在到达目标上方之前多了一段避障移动。

## 12. 调参与排查顺序

检查这条基本链路时，建议按以下顺序看问题：

1. 目标输入是否正确  
   查看 `target.position_board` 和 `target.position_world`，确认目标编号、板坐标、世界坐标一致。

2. hover/contact/hit 点是否合理  
   查看 `make_hit_keyposes()` 输出，确认 `z_hover > z_hit > z_hit - press_depth`。

3. IK 是否连续  
   如果关节角突然大幅跳变，优先检查 IK seed、工具姿态和目标点是否超出可达空间。

4. 轨迹是否平滑  
   查看 `traj.q_des/qd_des/qdd_des`，确认段时间 `segment_time`、采样周期 `dt` 和 `motion_mode` 是否符合期望。

5. 控制误差是否过大  
   基线版本看 `tracking.rmse` 和 `tracking.max_abs_error`；力控版本看末端 `x_des - x`、速度限幅和力矩限幅。

6. 敲击判定是否过严或过松  
   重点检查 `hit_threshold`、`threshold`、`min_contact_time` 和 `press_depth/max_press_depth`。

## 13. 一句话总结

这条链路的核心思想是：把外部目标点先归一化成世界坐标中的目标孔位，再用固定语义的 `hover/contact/press/back` 关键点描述敲击动作；每个关键点通过连续 IK 转成关节路点，路点之间用五次多项式平滑连接；执行层可以选择简单关节 PD 验证位置链路，也可以选择操作空间阻抗加接触力控制来模拟高速下压和击中过程。
