# Robotic Arm Whack-Moles

本仓库是机器人学课程大作业项目，用 UR5 机械臂完成“打地鼠”任务仿真。项目围绕目标板建模、目标选择、路径规划、轨迹生成、机械臂控制、接触力建模和 Python-MATLAB 通信逐步扩展。

仓库目前包含三条主要分支，三条分支的 README 使用同一份说明，方便从任意分支理解整个项目结构。

```text
main
  -> matlab-impact-force-control
      -> python-matlab-random-target-demo
```

## 分支总览

| 分支 | 定位 | 核心内容 | 适合场景 |
|---|---|---|---|
| `main` | 基础 MATLAB 主线分支 | UR5 建模、鼠标选点、RRT 避障、轨迹规划、PD 控制、随机目标仿真 | 展示基础机械臂规划控制流程 |
| `matlab-impact-force-control` | MATLAB 高速碰撞力控分支 | 接触力建模、冲击响应、导纳式力反馈、力控实验和报告数据 | 展示碰撞力控和实验分析 |
| `python-matlab-random-target-demo` | Python-MATLAB UDP 闭环 demo 分支 | Python 随机/视觉目标发送、MATLAB UDP 接收、实时力控敲击 | 展示完整实时闭环 demo |

## 分支一：`main`

### 分支定位

`main` 是基础 MATLAB 打地鼠主线分支，重点是完成 UR5 机械臂建模、目标板建模、鼠标点击目标、避障路径规划、关节轨迹生成、PD/重力补偿 PD 控制和仿真可视化。

该分支用于展示机械臂从“选定目标”到“规划路径”再到“执行敲击”的基础流程。

### 主要功能

- UR5 机械臂建模
- 目标板与九宫格孔位
- 鼠标点击选择目标
- 在线 RRT 避障规划
- 路径平滑
- IK 求解
- 关节空间轨迹规划
- PD / 重力补偿 PD 控制
- 2D 目标板与 3D 机器人可视化
- 随机目标单次打击仿真

### 运行入口与效果

#### 1. MATLAB 交互式打地鼠主线

```matlab
run("matlab/main_interactive_sim.m")
```

运行后会打开 2D 目标板窗口和 3D 机器人窗口。用户在目标板安全区域点击目标点后，程序会在线执行 RRT 避障规划、路径平滑、IK 求解、关节轨迹生成和控制仿真，机械臂会实时移动并完成敲击动作。

这个入口适合展示基础主流程：鼠标选点、避障规划、轨迹跟踪和机器人动画。

#### 2. MATLAB 随机目标单次仿真

```matlab
run("matlab/main_sim.m")
```

运行后程序会从九宫格孔位中随机选择一个目标，自动生成 `hover`、`contact`、`press`、`back` 等关键点，完成一次机械臂敲击仿真，并输出轨迹、结果和动画。

这个入口适合快速检查基础打击流程是否正常，也适合作为非交互演示。

## 分支二：`matlab-impact-force-control`

### 分支定位

`matlab-impact-force-control` 是 MATLAB-only 高速碰撞力控实验分支，专门用于测试机械臂与地鼠发生高速碰撞时的接触力建模、冲击响应和接触后的力反馈控制。

该分支保留基础 MATLAB 打地鼠流程，并扩展了接触力模型、操作空间阻抗控制、导纳式力反馈、地鼠下压可视化和实验结果生成。

### 主要功能

- 高速敲击接触力建模
- 地鼠圆柱体下压效果
- 操作空间阻抗控制
- 接触后导纳式力反馈
- 交互式点击目标力控敲击
- 多组报告实验
- 力曲线、峰值力、穿透深度、末端速度和关节力矩输出

### 运行入口与效果

#### 1. 单次高速碰撞力控实验

```matlab
run("matlab/main_impact_force_control.m")
```

运行后 MATLAB 会随机选择一个地鼠孔位，执行一次高速下压、接触碰撞、导纳式力反馈保持和回撤动作。程序会生成接触力曲线、末端速度、穿透深度、动画 GIF 和结果 JSON。

输出结果通常包括：

```text
results/figures/impact_force_control_results.png
results/videos/impact_force_control_animation.gif
shared/impact_force_control_result.json
```

这个入口适合展示高速碰撞力控的核心效果。

#### 2. 交互式高速碰撞力控实验

```matlab
run("matlab/main_interactive_impact_force_control.m")
```

运行后会打开目标板窗口和机器人窗口。用户点击目标点后，机械臂会立即从当前状态规划到目标并执行力控敲击；新的点击会抢占当前动作。界面会实时显示接触力、滤波力、峰值力和地鼠下压效果。

这个入口适合课堂演示和人工交互测试。

#### 3. 报告实验批量生成

```matlab
run("matlab/experiments/run_impact_force_experiments.m")
```

运行后会自动执行多组实验，包括典型单次敲击、不同敲击速度对峰值力的影响、有无导纳反馈对比、不同击倒阈值对比等，并生成报告用图表和数据。

实验说明和结果一般位于：

```text
matlab/experiments/impact_force_experiments.md
matlab/experiments/results/
```

这个入口适合课程报告的数据分析部分，不适合作为实时 demo 的第一选择。



## 分支三：`python-matlab-random-target-demo`

### 分支定位

`python-matlab-random-target-demo` 是基于 `matlab-impact-force-control` 扩展出的 Python-MATLAB UDP 闭环演示分支，用于打通“Python 实时目标输入 -> MATLAB 接收目标 -> 现有轨迹规划与高速碰撞力控敲打 -> 可视化显示结果”的完整流程。

当前支持两个输入源：Python 随机九宫格目标发送，以及视觉识别程序输出九宫格编号后的 UDP 桥接发送。

### 主要功能

- MATLAB UDP 实时接收目标
- Python 随机九宫格目标发送
- Python 视觉识别结果桥接
- `target_id=1..9` 九宫格目标协议
- MATLAB 端实时抢占当前动作并敲击
- 保留高速碰撞力控可视化和地鼠下压效果

### 运行入口与效果

#### 1. 启动 MATLAB UDP 接收与实时敲击主程序

```matlab
run("matlab/main_udp_impact_force_control.m")
```

运行后 MATLAB 会打开目标板窗口和机器人窗口，并开始监听 UDP 目标消息。此时 MATLAB 会等待 Python 发送目标，不会自己随机选择目标。

这个命令需要先运行，然后再启动下面任意一个 Python 发送端。

#### 2. Python 随机九宫格目标发送

```powershell
python -m python.communication.random_board_target_sender --host 127.0.0.1 --port 5005 --period-s 2.5
```

运行后 Python 会每隔 2.5 秒随机选择一个 `target_id=1..9`，通过 UDP 发送给 MATLAB。MATLAB 收到后会实时更新目标点，并控制机械臂执行高速力控敲击。

这个入口适合测试 Python-MATLAB UDP 通信是否正常。有限次调试可以使用：

```powershell
python -m python.communication.random_board_target_sender --count 3 --seed 7
```

#### 3. Python 视觉识别目标桥接

```powershell
python -m python.communication.vision_grid_target_sender
```

运行后 Python 会调用视觉识别程序，读取识别出的九宫格目标编号，并通过 UDP 发送给 MATLAB。MATLAB 收到目标后执行实时力控敲击。

这个入口适合展示完整闭环：视觉识别目标 -> Python 发送 UDP -> MATLAB 接收 -> 机械臂敲击。

如果要把参数传给视觉脚本，可以追加在命令末尾，例如：

```powershell
python -m python.communication.vision_grid_target_sender -- --index 1 --backend msmf --fourcc YUY2 --hough-lines --hough-circles
```


停止方式：

```text
MATLAB: 在目标板窗口按 Esc，或关闭两个图窗
Python: Ctrl+C
```

## 如何选择分支

- 只想看基础打地鼠规划、RRT 避障、轨迹跟踪和 MATLAB 可视化：使用 `main`
- 想看高速碰撞、接触力、力反馈控制和报告实验：使用 `matlab-impact-force-control`
- 想跑 Python 输入目标、MATLAB 实时接收并敲击的完整 demo：使用 `python-matlab-random-target-demo`

常用切换命令：

```powershell
git switch main
git switch matlab-impact-force-control
git switch python-matlab-random-target-demo
```

如果本地还没有第三条分支，可以先执行：

```powershell
git fetch origin
git switch -c python-matlab-random-target-demo --track origin/python-matlab-random-target-demo
```

## 运行环境

### MATLAB

项目主要依赖 MATLAB，并使用 Robotics System Toolbox 完成 UR5 机械臂建模、运动学求解和动力学相关计算。

建议在 MATLAB 中先进入仓库根目录，再运行对应脚本：

```matlab
cd("你的仓库路径")
run("matlab/main_sim.m")
```

### Python

`python-matlab-random-target-demo` 分支需要 Python 来发送 UDP 目标；`main` 分支和 `matlab-impact-force-control` 分支的核心演示可以只运行 MATLAB。

如果当前分支包含 `requirements.txt`，可以安装依赖：

```powershell
pip install -r requirements.txt
```

如果当前分支没有 `requirements.txt`，则以该分支实际包含的 Python 文件和脚本说明为准。

## 数据输出与通信方式

### MATLAB 结果输出

部分 MATLAB 入口会输出：

```text
shared/result.json
shared/q_traj.csv
shared/impact_force_control_result.json
results/figures/
results/videos/
```

其中 `result.json` 或 `impact_force_control_result.json` 保存敲击结果摘要，`q_traj.csv` 保存关节轨迹数据，`results/figures/` 和 `results/videos/` 保存图像与动画。

### UDP 目标通信

`python-matlab-random-target-demo` 分支使用 UDP JSON datagram 从 Python 向 MATLAB 发送目标。典型 payload 如下：

```json
{
  "valid": true,
  "source": "random_board_demo",
  "target_id": 5,
  "row": 2,
  "col": 2,
  "board": [0.0, 0.0],
  "world": [0.45, 0.0, 0.05]
}
```

MATLAB 端优先使用 `target_id`，也兼容 `id`、`row/col`、`board` 或 `world` 字段。若连续收到多个 UDP 包，主循环会清空积压包并只处理最新的合法目标。

九宫格编号遵循 `config_board.m` 中的行优先顺序：

```text
1 2 3   y = -0.13
4 5 6   y =  0.00
7 8 9   y =  0.13
```

## 主要目录说明

```text
matlab/      MATLAB 建模、规划、控制、可视化与实验代码
python/      Python 通信、视觉桥接或其他辅助代码
shared/      MATLAB 与 Python 共享的 JSON/CSV 数据
docs/        原理说明、接口说明和实验说明
results/     图像、动画、实验数据等输出
data/        标定、模型或其他输入数据
```

不同分支实际包含的目录和文件不完全相同。如果某个命令在当前分支不可用，请先确认是否切换到了对应分支。

## 核心控制思想

基础主线中，目标点经过路径规划和 IK 求解后生成关节空间轨迹，再由 PD 或重力补偿 PD 控制器跟踪执行。

高速碰撞力控分支中，接触力在每个仿真步作为外力矩进入关节动力学：

```text
M(q) qdd + C(q,qd) + G(q) = tau_cmd + J(q)' F_contact
```

控制器使用操作空间阻抗控制：

```text
F_task = Kx (x_cmd - x) + Dx (xd_cmd - xd)
tau_cmd = Jv' F_task + tau_posture + G(q)
```

接触后，力误差通过一阶导纳近似修正末端法向位置：

```text
z_cmd = z_cmd - K_adm (F_des - F_filtered) dt
```

高速碰撞分为四个阶段：

```text
approach: 末端从 hover 点向地鼠上方接近
impact:   以设定速度向下接触地鼠
force:    接触后使用导纳式力反馈修正 z 方向命令
retract:  达到阈值并保持足够时间后回撤
```

## 注意事项

- 三条分支共用这份 README，但不同分支实际包含的文件不同。
- `main` 分支介绍的是基础 MATLAB 打地鼠流程，不包含已舍弃的颜色识别和视觉目标读取流程。
- `matlab-impact-force-control` 分支主要用于 MATLAB 力控实验，通常不需要 Python。
- `python-matlab-random-target-demo` 分支需要 MATLAB 端先监听 UDP，再启动 Python 发送端。
- 运行脚本前请确认 MATLAB 或终端当前目录位于仓库根目录。
