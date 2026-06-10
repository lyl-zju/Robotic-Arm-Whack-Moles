# 机械臂打地鼠项目骨架（不含 YOLO）

本项目根据两份实施文档生成，覆盖课程大作业主线中的前两部分：

1. MATLAB 机械臂打地鼠主线：机械臂建模、目标板、随机/鼠标目标、敲击关键位姿、关节轨迹、RRT 避障、PD 控制和结果分析。
2. PyBullet + OpenCV 视觉保底方案：PyBullet 目标板与相机、红色目标颜色识别、像素到目标板坐标映射、`target.json` 文件通信。

本骨架故意不包含 YOLO 训练、标注、推理和数据集目录。后续若时间充足，可在现有 `target.json` 接口后面替换检测来源，MATLAB 主线不需要改。

## 目录结构

```text
robot_whac_a_mole_no_yolo/
  README.md
  requirements.txt
  python/
    config.py
    pybullet_scene.py
    camera_capture.py
    detect_color_target.py
    homography.py
    write_target_json.py
    run_color_pipeline.py
    replay_matlab_traj.py
  matlab/
    main_sim.m
    main_hit_from_vision.m
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
    screenshots/
  results/
    figures/
    videos/
    data/
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
python python/run_color_pipeline.py --gui
```

无界面运行：

```bash
python python/run_color_pipeline.py --nogui
```

输出文件：

- `data/screenshots/camera_rgb.png`
- `data/screenshots/color_detection_annotated.png`
- `shared/target.json`

### 2. MATLAB 主线侧

在 MATLAB 中进入项目根目录后运行：

```matlab
run("matlab/main_sim.m")
```

随机目标敲击主线：

- 加载 UR5 或本地机械臂模型
- 建立 3x3 目标板
- `main_sim.m` 调用 `generate_random_target(cfgBoard)`，从 3x3 孔位中随机抽取目标点
- 规划 `P_hover -> P_hit -> P_back`
- 生成关节轨迹
- 用简化 PD 模型仿真跟踪
- 绘制误差图并保存结果

读取 Python 视觉结果并敲击：

```matlab
run("matlab/main_hit_from_vision.m")
```

## 打击链路

当前工程实现的是“识别或生成目标点后进行打击”的完整仿真链路，不包含夹爪开合和真实抓取动作。

### 随机目标链路

`matlab/main_sim.m` 是随机目标仿真入口：

1. `main_sim.m` 加载 `config_robot.m`、`config_board.m`、`config_controller.m`，并通过 `build_robot.m` 建立机械臂模型。
2. `main_sim.m` 调用 `generate_random_target(cfgBoard)`，从 `cfgBoard.holes_board` / `cfgBoard.holes_world` 的 3x3 孔位中随机抽取目标点。
3. `execute_hit_target.m` 接收 `target.position_world`，调用 `make_hit_keyposes.m` 生成 `hover`、`hit`、`back` 三个关键打击点。
4. `solve_ik.m` 使用 Robotics System Toolbox 的 `inverseKinematics`，依次求解 `qHover`、`qHit`、`qBack`。
5. `plan_joint_traj.m` 将 `[q0; qHover; qHit; qBack]` 转换为关节空间五次多项式轨迹，输出 `q_des`、`qd_des`、`qdd_des`。
6. `simulate_joint_tracking.m` 使用 `config_controller.m` 中的 PD 参数进行简化关节跟踪仿真。
7. `main_sim.m` 保存 `shared/result.json`、`shared/q_traj.csv`，并生成轨迹图和动画。

### 视觉目标链路

`python/run_color_pipeline.py` 与 `matlab/main_hit_from_vision.m` 组成视觉驱动入口：

1. `run_color_pipeline.py` 通过 PyBullet 截图或读取输入图片获得相机图像。
2. `detect_color_target.py` 在图像中检测红色目标，得到目标像素中心。
3. `homography.py` 根据 `data/calibration/board_corners.json` 将像素坐标映射为目标板坐标。
4. `write_target_json.py` 将 `board` 坐标和 `world` 坐标写入 `shared/target.json`。
5. `main_hit_from_vision.m` 读取 `shared/target.json`，优先使用 `world` 坐标；若只有 `board` 坐标，则调用 `board_to_world.m` 转换。
6. 后续同样进入 `execute_hit_target.m -> make_hit_keyposes.m -> solve_ik.m -> plan_joint_traj.m -> simulate_joint_tracking.m`。
7. `main_hit_from_vision.m` 保存 `shared/result.json`、`shared/q_traj.csv`，并生成视觉目标跟踪结果图。

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

MATLAB 可选写给 PyBullet：

```csv
t,q1,q2,q3,q4,q5,q6
0.00,0.1,-0.3,0.5,0.0,0.4,0.0
```

MATLAB 结果：

```json
{
  "hit_success": true,
  "hit_error": 0.008,
  "response_time": 1.24,
  "controller": "PD",
  "path_type": "straight"
}
```

## 坐标约定

- 目标板尺寸：`0.60 m x 0.40 m`
- 孔位：`3 x 3`
- 目标板坐标系 `{G}` 原点：目标板中心
- 孔位横向：`x = [-0.15, 0, 0.15]`
- 孔位纵向：`y = [-0.10, 0, 0.10]`
- MATLAB 机械臂基坐标系 `{B}` 下目标板中心：默认 `[0.45, 0, 0.05]`
- 简化变换：`P_B = R_BG * P_G + p_BG`，默认 `R_BG = I`

如果 PyBullet 截图中的像素映射误差较大，先修改 `data/calibration/board_corners.json` 中四个角点的图像坐标。

## 当前骨架完成度

- 已生成 PyBullet 场景与颜色识别主线代码。
- 已生成 MATLAB 主线模块、函数接口和基础可运行逻辑。
- 已保留 RRT、路径平滑、PD 仿真、结果绘图和文件通信接口。
- 未创建 YOLO 模块，避免把项目重点带偏。
