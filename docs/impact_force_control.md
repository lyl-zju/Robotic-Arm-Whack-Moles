# 高速碰撞瞬间力控实验说明

本文档说明本分支中 MATLAB 高速碰撞力控实验的原理、实现方式和完整运行流程。

## 1. 设计目标

目标不是简单地在轨迹结束后判断是否击中，而是在 MATLAB 仿真循环中实时模拟：

- UR5 末端高速接近地鼠；
- 末端与地鼠发生接触后的冲击力；
- 接触力对机械臂动力学的反作用；
- 接触后的力反馈调节；
- 当接触力超过阈值并保持足够时间后判定击倒。

运行入口：

```matlab
run("matlab/main_impact_force_control.m")
```

鼠标选目标点的交互入口：

```matlab
run("matlab/main_interactive_impact_force_control.m")
```

核心实现：

```text
matlab/config/config_impact_force_control.m
matlab/control/simulate_impact_force_control.m
matlab/visualization/plot_impact_force_control.m
matlab/main_interactive_impact_force_control.m
```

## 2. 原理

### 2.1 为什么不能只靠反馈控制碰撞第一瞬间

高速碰撞的峰值力通常在毫秒级时间内出现。真实机械臂中力传感器、滤波、控制计算和电机响应都有延迟，因此反馈控制看到峰值时，峰值往往已经发生。

所以更合理的策略是：

```text
碰撞前：规划接触速度、姿态、等效刚度和阻尼
碰撞中：通过阻抗和接触模型限制冲击
碰撞后：使用力反馈调节持续接触力
```

本实验采用这种混合策略，而不是声称能精确闭环控制第一帧冲击峰值。

### 2.2 接触动力学

接触只考虑地鼠表面的法向方向，也就是世界坐标系 z 方向。若末端低于地鼠顶面：

```text
delta = max(0, z_hit - z_tip)
v_n = max(0, -z_dot_tip)
```

其中 `delta` 是穿透深度，`v_n` 是闭合速度。

默认接触模型采用 Hunt-Crossley 型非线性模型：

```text
F_contact = k * delta^n + c * delta^n * v_n
```

参数在：

```text
matlab/config/config_impact_force_control.m
```

主要参数：

```matlab
cfg.contact_stiffness = 120000.0;
cfg.contact_damping = 9000.0;
cfg.contact_exponent = 1.5;
cfg.max_contact_force = 90.0;
```

`max_contact_force` 是安全限幅，避免数值积分时出现不现实的力尖峰。

### 2.3 关节动力学

旧流程里的 `simulate_contact_force.m` 是事后根据末端轨迹估计力；本分支的新实现把接触力放进每个仿真步的动力学方程：

```text
M(q) qdd + C(q,qd) + G(q) = tau_cmd + J(q)' F_contact
```

在代码中对应：

```matlab
mass = massMatrix(robot, q.');
coriolis = velocityProduct(robot, q.', qd.').';
tauGravity = gravityTorque(robot, q.').';
tauContact = J.' * contactWrench;
qdd = mass \ (tauCommand + tauContact - coriolis - tauGravity - tauDamping);
```

这里 `F_contact` 是地鼠作用在机械臂末端上的向上反作用力，经过雅可比矩阵转置映射成关节外力矩。

### 2.4 操作空间阻抗控制

末端位置控制不再直接使用关节空间 PD，而是在操作空间计算期望力：

```text
F_task = Kx (x_cmd - x) + Dx (xd_cmd - xd)
```

再映射到关节力矩：

```text
tau_task = Jv' F_task
```

代码中还加了轻量姿态保持项：

```text
tau_posture = Kq (q_nominal - q) - Dq qd
```

最终命令力矩：

```text
tau_cmd = tau_task + tau_posture + G(q)
```

并加入关节力矩限幅、关节速度限幅和关节阻尼，用来模拟真实机械臂执行器限制。

### 2.5 接触后的导纳式力反馈

接触发生后，控制目标从“继续按固定轨迹下压”变为“让接触力达到目标力”。

目标力设置为阈值上方：

```matlab
F_des = cfgForce.threshold * cfgImpact.force_target_ratio;
```

力传感器信号经过延迟、噪声和一阶低通滤波：

```text
F_raw -> sensor delay -> noise -> low-pass filter -> F_filtered
```

力误差：

```text
F_err = F_des - F_filtered
```

再通过一阶导纳近似修正 z 方向命令：

```text
z_cmd = z_cmd - K_adm * F_err * dt
```

解释：

- 如果力小于目标力，`F_err > 0`，`z_cmd` 变小，末端继续下压；
- 如果力大于目标力，`F_err < 0`，`z_cmd` 变大，末端向上释放；
- `z_cmd` 会被限制在安全下压深度内。

这不是精确控制碰撞第一瞬间峰值，而是控制接触后的持续法向力。

## 3. 状态机

控制流程由 `simulate_impact_force_control.m` 内部状态机驱动：

```text
approach -> force -> retract
```

### approach

末端从目标上方 `z_hover` 运动到接触前高度：

```text
z_hit + preimpact_clearance
```

之后以给定高速下压：

```matlab
cfg.impact_speed = 0.34;
```

### force

检测到接触后进入力反馈阶段：

```matlab
forceRaw >= cfg.contact_detect_force || penetration > 0
```

此阶段使用接触刚度较低、阻尼较高的阻抗参数，并启用导纳式 z 命令修正。

### retract

当接触力超过阈值并累计保持足够时间：

```matlab
cfgForce.min_contact_time = 0.02;
```

或者力控阶段超时后，机械臂回撤到 hover 高度。

## 4. 输出结果

运行后保存：

```text
shared/impact_force_control_result.json
results/figures/impact_force_control_results.png
results/videos/impact_force_control_animation.gif
```

交互式入口为了保证鼠标连续选点时的实时性，不再把每一次点击离线仿真后播放，也不在交互循环里写 GIF。它直接在主循环中积分当前机械臂状态，新的点击会立即抢占当前轨迹并从当前 `q/qd` 重新规划。

结果 JSON 中包含：

- `hit_success`：是否成功击倒；
- `force_threshold`：击倒阈值；
- `force_target`：力控目标力；
- `peak_force`：峰值接触力；
- `contact_duration`：接触总时长；
- `threshold_duration`：超过阈值的累计时长；
- `max_penetration`：最大穿透深度；
- `first_contact_time`：第一次接触时刻；
- `success_time`：达到击倒判据的时刻；
- `impact_speed`：碰撞前设定下压速度；
- `contact_model`：接触模型。

图像中包含：

- 末端实际位置和命令位置；
- 原始接触力、滤波接触力和目标力；
- 穿透深度；
- z 方向末端速度；
- 关节力矩范数和控制阶段。

动画 `impact_force_control_animation.gif` 中，目标不再只是平面点，而是一个红色圆柱地鼠。接触阶段会根据穿透深度显示轻微下压；达到击倒判据后，地鼠会继续下沉到洞口中。场景左上角还会实时显示原始接触力、滤波接触力、峰值力、阈值和控制阶段，并用红/绿力条表示是否超过阈值。

## 5. 调参建议

优先调这些参数：

```matlab
cfg.impact_speed
cfg.contact_stiffness
cfg.contact_damping
cfg.force_target_ratio
cfg.admittance_gain
cfg.max_press_depth
```

调参顺序建议：

1. 先固定 `admittance_gain`，调 `impact_speed` 和 `contact_stiffness`，让接触峰值能达到阈值附近。
2. 再调 `contact_damping`，减少过尖锐的冲击峰值。
3. 再调 `admittance_gain`，让接触后的持续力接近目标力。
4. 最后调 `max_press_depth` 和 `max_contact_force`，避免不现实的过大穿透或过大力。

如果曲线振荡明显，通常降低：

```matlab
cfg.admittance_gain
cfg.k_xyz_contact(3)
```

或提高：

```matlab
cfg.d_xyz_contact(3)
cfg.force_filter_time_constant
```

## 6. 与原 MATLAB 基线的区别

原 `main_sim.m` 流程：

```text
规划 hover/contact/press/back 关键点
关节空间轨迹
简化 PD 跟踪
事后估计虚拟接触力
```

新 `main_impact_force_control.m` 流程：

```text
从 hover 姿态开始高速下压
每步计算接触力
接触力作为外力矩进入关节动力学
操作空间阻抗控制
接触后导纳式力反馈
成功后回撤
```

因此新流程更适合用来说明“真实机械臂高速碰撞瞬间力控”的建模思路。

## 7. 鼠标选点力控演示

运行：

```matlab
run("matlab/main_interactive_impact_force_control.m")
```

操作流程：

1. MATLAB 打开 2D 目标板窗口。
2. 在目标板范围内点击一个目标点。
3. 程序将点击点转换为世界坐标。
4. 主循环从当前实际关节状态 `q/qd` 出发，直接规划到目标 `hover/contact/hit/back` 关键点，并用五次多项式连接关节轨迹。
5. 动画窗口实时显示机械臂敲击、地鼠下压和接触力 HUD。
6. 可以在敲击过程中继续点击目标点；新的点击会立即抢占当前轨迹，不会等待上一轮敲击结束。
7. 按 Enter/Esc 或关闭目标板窗口退出。
