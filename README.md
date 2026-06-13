# 机械臂打地鼠项目（不含 YOLO）

本项目覆盖课程大作业主线中的两部分：

1. MATLAB 机械臂打地鼠主线：UR5 建模、目标板、鼠标选点目标、在线 RRT 避障、动态轨迹抢占重规划、关节轨迹、PD/重力补偿 PD 控制和结果分析。
2. PyBullet + OpenCV 视觉保底方案：PyBullet 目标板与相机、红色目标颜色识别、像素到目标板坐标映射、`target.json` 文件通信。

本项目故意不包含 YOLO 训练、标注、推理和数据集目录。后续若时间充足，可在现有 `target.json` 接口后面替换检测来源，MATLAB 主线不需要大改。

## 目录结构

```text
Robotic-Arm-Whack-Moles/
  README.md
  requirements.txt
  python/
    README.md
    common/
    vision/
    simulation/
    io_utils/
    run_color_pipeline.py
    replay_matlab_force_hit.py
  matlab/
    main_interactive_sim.m
    main_sim.m
    main_hit_from_vision.m
    test_load_ur5.m
    test_ur5_kinematics.m
    config/
    robot/
    target/
    planning/
    control/
    visualization/
    io/
  shared/
    target.json
    result.json
    q_traj.csv
  data/
    calibration/board_corners.json
  results/
    figures/
    videos/
  docs/
    interface_spec.md
    dev_checklist.md
```

## 推荐运行顺序

### 1. Python 视觉侧

安装依赖：

```bash
pip install -r requirements.txt
```

生成 PyBullet 场景、截图、颜色识别并写入共享目标：

```bash
python -m python.vision.run_color_pipeline --gui
```

无界面运行：

```bash
python -m python.vision.run_color_pipeline --nogui
```


输出文件：

- `data/screenshots/camera_rgb.png`
- `data/screenshots/color_detection_annotated.png`
- `shared/target.json`

读取 Python 视觉结果并由 MATLAB 执行一次敲击：

```matlab
run("matlab/main_hit_from_vision.m")
```

### 2. MATLAB 交互式 RRT 主线

在 MATLAB 中进入项目根目录后运行：

```matlab
run("matlab/main_interactive_sim.m")
```

这是当前推荐的主仿真入口。运行后会打开两个窗口：

- 2D 目标板窗口：显示目标板、孔位、RRT 障碍物、目标点、末端投影和 RRT 路径。
- 3D 机器人窗口：显示 UR5、目标板、障碍物、RRT 悬停路径、打击关键点和末端轨迹。

交互逻辑：

- 在目标板安全区域点击，系统会从当前实际末端位置在线运行 RRT，平滑路径，逐点求 IK，并抢占当前轨迹重新执行。
- 如果点击点落在障碍物内部，输入会被拒绝，命令行和窗口标题会给出警告，机械臂继续执行上一条有效轨迹。
- 关闭任一窗口，或在 2D 目标板窗口按 `Esc`，停止仿真。

## 打击链路

当前工程实现的是“目标点生成或识别后进行敲击”的仿真链路，不包含夹爪开合和真实抓取动作。

### 鼠标选点 + RRT 避障链路

`matlab/main_interactive_sim.m` 是当前主入口：

1. 脚本加载 `config_robot.m`、`config_board.m`、`config_controller.m`，并通过 `build_robot.m` 建立 UR5 模型。
2. 脚本定义目标板 2D 坐标系下的障碍物，并调用 `draw_board.m` 和 `draw_obstacles.m` 绘制 2D 交互窗口。
3. 鼠标点击回调读取点击点 `[x, y]`，先调用 `collision_check_2d(clicked_point, obstacles)` 做目标合法性校验。
4. 若点击点在障碍物内部，则拒绝该输入，不设置新目标，不打断当前轨迹。
5. 若点击点合法，则调用 `board_to_world.m` 得到目标世界坐标，并设置 `is_new_target = true`。
6. 主循环以 `dt = 0.01 s` 步进运行。检测到新目标后，截取当前实际关节角 `q_act` 和关节速度 `qd_act`。
7. 脚本通过正运动学 `getTransform()` 获取当前末端世界坐标，并投影回目标板坐标，作为 RRT 起点。
8. 调用 `rrt_plan_2d.m` 在线规划二维避障路径，再调用 `smooth_path.m` 剪枝平滑。
9. 将 RRT 的 2D 路点通过 `board_to_world.m` 转成 3D 世界坐标，并统一提升到 `cfgBoard.z_hover` 高度。
10. 对每个 3D 路点调用 `solve_ik.m` 求关节路点，最后追加 `hit` 和 `back` 动作。
11. 调用扩展后的 `plan_joint_traj.m` 生成多段五次多项式关节轨迹。新轨迹起点速度使用抢占瞬间的 `qd_act`，末端速度为 0。
12. 每个仿真步调用 `gravity_pd_controller.m`，再用简化关节动力学积分更新 `q_act` 和 `qd_act`，并实时刷新 2D/3D 可视化。

### 视觉目标链路

`python -m python.vision.run_color_pipeline` 与 `matlab/main_hit_from_vision.m` 组成视觉驱动入口：

1. `python/vision/run_color_pipeline.py` 通过 PyBullet 截图或读取输入图片获得相机图像。
2. `python/vision/detect_color_target.py` 在图像中检测红色目标，得到目标像素中心。
3. `python/vision/homography.py` 根据 `data/calibration/board_corners.json` 将像素坐标映射为目标板坐标。
4. `python/io_utils/write_target_json.py` 将 `board` 坐标和 `world` 坐标写入 `shared/target.json`。
5. `main_hit_from_vision.m` 读取 `shared/target.json`，优先使用 `world` 坐标；若只有 `board` 坐标，则调用 `board_to_world.m` 转换。
6. 后续进入 `execute_hit_target.m -> make_hit_keyposes.m -> solve_ik.m -> plan_joint_traj.m -> simulate_joint_tracking.m`。
7. `main_hit_from_vision.m` 保存 `shared/result.json`、`shared/q_traj.csv`，并生成视觉目标跟踪结果图。

### 打击轨迹模式

随机目标、交互式目标和视觉目标入口都使用同一个轨迹模式开关 `cfgController.motion_mode`：

- `hover_stop`：默认模式。机械臂到达目标上方 hover 点时速度置零，然后再下捶。
- `continuous`：连续模式。机械臂经过 hover 点不停车，直接连贯下捶。

在 MATLAB 命令中可以用变量选择模式：

```matlab
hitMotionMode = "hover_stop";
run("matlab/main_sim.m")
```

```matlab
hitMotionMode = "continuous";
run("matlab/main_interactive_sim.m")
```

视觉入口同理：

```matlab
hitMotionMode = "continuous";
run("matlab/main_hit_from_vision.m")
```

也可以用环境变量：

```matlab
setenv("HIT_MOTION_MODE", "continuous");
run("matlab/main_sim.m")
```

### 随机目标链路

`matlab/main_sim.m` 是保留的单次随机目标仿真入口，主要用于回归测试和生成演示 GIF：

1. `main_sim.m` 从 `cfgBoard.holes_board` / `cfgBoard.holes_world` 的 3x3 孔位中随机抽取目标点。
2. `execute_hit_target.m` 接收 `target.position_world`，调用 `make_hit_keyposes.m` 生成 `hover`、`contact`、`press`、`back` 四个关键打击点。
3. `solve_ik.m` 使用 Robotics System Toolbox 的 `inverseKinematics` 依次求解关键点关节角。
4. `make_hit_motion_boundaries.m` 根据 `motion_mode` 生成速度边界，再由 `plan_joint_traj.m` 生成关节空间五次多项式轨迹。
5. `simulate_joint_tracking.m` 使用简化 PD 模型进行关节跟踪仿真。
6. `main_sim.m` 保存 `shared/result.json`、`shared/q_traj.csv`，并生成轨迹图和动画。

## 共享数据接口

Python 写给 MATLAB：

```json
{
  "valid": true,
  "source": "color",
  "confidence": 0.98,
  "pixel": [325.0, 241.0],
  "board": [0.12, -0.05],
  "world": [0.57, -0.05, 0.05],
  "timestamp": 2.35
}
```

MATLAB 导出的关节轨迹：

```csv
t,q1,q2,q3,q4,q5,q6
0.00,0.1,-0.3,0.5,0.0,0.4,0.0
```

MATLAB 结果摘要：

```json
{
  "hit_success": true,
  "hit_error": 0.008,
  "response_time": 1.24,
  "controller": "PD",
  "path_type": "rrt"
}
```

说明：

- `main_sim.m` 和 `main_hit_from_vision.m` 会导出 `shared/result.json` 与 `shared/q_traj.csv`。
- `main_interactive_sim.m` 当前重点是交互式实时仿真和可视化，不会自动覆盖 `shared/result.json` 或 `shared/q_traj.csv`。
- `python -m python.simulation.replay_matlab_force_hit` 可读取 MATLAB 导出的 `q_traj.csv`，在 PyBullet 中加载 UR5 并实时检测打击接触力。

## 坐标约定

- 目标板尺寸：`0.60 m x 0.40 m`
- 孔位：`3 x 3`
- 目标板坐标系 `{G}` 原点：目标板中心
- 孔位横向：`x = [-0.20, 0, 0.20]`
- 孔位纵向：`y = [-0.13, 0, 0.13]`
- MATLAB 机械臂基坐标系 `{B}` 下目标板中心：默认 `[0.45, 0, 0.05]`
- 简化变换：`P_B = R_BG * P_G + p_BG`，默认 `R_BG = I`
- Python 侧 `python/common/config.py` 已与 MATLAB 的目标板尺寸和孔位坐标保持一致。
- `data/calibration/board_corners.json` 中的 `board_points` 已按板四角设置为 `[-0.3, 0.2]`、`[0.3, 0.2]`、`[0.3, -0.2]`、`[-0.3, -0.2]`。

如果 PyBullet 截图中的像素映射误差较大，先修改 `data/calibration/board_corners.json` 中四个角点的图像坐标 `image_points`。

## 当前完成度

- 已生成 PyBullet 场景与颜色识别主线代码。
- 已生成 MATLAB UR5 建模、IK、轨迹规划、控制仿真、结果绘图和文件通信接口。
- 已完成交互式 `main_interactive_sim.m`：鼠标点击目标、障碍物点击拦截、动态轨迹抢占、在线 2D RRT、路径平滑、多段 IK、速度边界连续的关节轨迹和实时 2D/3D 可视化。
- `collision_check_2d.m` 已支持点碰撞检测和线段碰撞检测两种调用形式。
- `plan_joint_traj.m` 已支持可选速度/加速度路点，兼容旧三参数调用。
- 已保留随机目标入口 `main_sim.m` 和视觉目标入口 `main_hit_from_vision.m` 作为回归测试/集成入口；随机、交互和视觉入口均支持 `hover_stop` / `continuous` 两种打击轨迹模式。
- 未创建 YOLO 模块，避免把项目重点带偏。
- 已新增 PyBullet UR5 力检测回放入口 `python -m python.simulation.replay_matlab_force_hit`。
